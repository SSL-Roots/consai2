#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <consai_msgs/VisionPacket.h>
#include <consai_msgs/VisionData.h>
#include <consai2_msgs/RobotInfo.h>
#include <consai2_msgs/BallInfo.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <sstream>
#include <cmath>

#include <world_observer/world_observer_ros.hpp>
#include <world_observer/estimator.hpp>
#include <world_observer/enemy_estimator.hpp>
#include <world_observer/ball_estimator.hpp>

// ObserverBase クラス
// ロボットまたはボールの位置および速度推定・存在判定を担うクラスのベースクラス
class ObserverBase
{
public:
    ObserverBase() : 
        DISAPPEAR_THREASHOLD_TIME(3.0),
        latest_observed_time_(ros::Time(0))
    {
        this->estimator_.Init(0.016);
    }

    ObserverBase (const ObserverBase& obj):
        DISAPPEAR_THREASHOLD_TIME(obj.DISAPPEAR_THREASHOLD_TIME),
        latest_observed_time_(obj.latest_observed_time_)
    {
    }

    void update()
    {
        this->detected_ = false;

        if (this->DoesDisappeared())
        {
            this->disappeared_ = true;
            this->estimator_.Reset();
        }
        else
        {
            this->odom_ = this->estimator_.estimate();
        }
    }

    void update(std::vector<geometry2d::Pose> observations)
    {
        if (observations.size() == 0)
        {
            this->update();
            return;
        }

        this->detected_ = true;
        this->latest_observed_time_ = ros::Time::now();
        this->odom_ = this->estimator_.estimate(observations);

        // consai2_msgs/RobotInfoへの変換用に保存
        this->last_detection_pose = observations[0];
    }

protected:
    geometry2d::Odometry odom_;
    bool detected_;
    bool disappeared_;
    geometry2d::Pose last_detection_pose;

    EnemyEstimator estimator_;

    ros::Duration DISAPPEAR_THREASHOLD_TIME;
    ros::Time latest_observed_time_;

    bool DoesDisappeared()
    {
        if ((ros::Time::now() - this->latest_observed_time_) > this->DISAPPEAR_THREASHOLD_TIME)
        {
            return true;
        }
        return false;
    }
};

// RobotObserver クラス
//
// ロボット単体の位置及び速度推定・存在判定を担うクラス
class RobotObserver : public ObserverBase
{
public:
    RobotObserver(int robot_id) :
        robot_id_(robot_id)
    {}

    RobotObserver(const RobotObserver& obj) :
        robot_id_(obj.robot_id_)
    {}

    consai2_msgs::RobotInfo ToRosMsg()
    {
        consai2_msgs::RobotInfo msg;

        msg.robot_id = this->robot_id_;
        msg.pose = this->odom_.pose.ToROSPose2D();

        msg.velocity.x = this->odom_.velocity.x;
        msg.velocity.y = this->odom_.velocity.y;
        msg.velocity.theta = this->odom_.velocity.theta;

        msg.velocity_twist = this->odom_.velocity.ToROSTwist();

        msg.detected = this->detected_;
        msg.detection_stamp = this->latest_observed_time_;
        msg.disappeared = this->disappeared_;

        msg.last_detection_pose = this->last_detection_pose.ToROSPose2D();
    }

private:
    int robot_id_;
};

// BallObserver クラス
//
// ボール単体の位置及び速度推定・存在判定を担うクラス
class BallObserver : public ObserverBase
{
public:
    consai2_msgs::BallInfo ToRosMsg()
    {
        consai2_msgs::BallInfo msg;

        msg.pose = this->odom_.pose.ToROSPose2D();

        msg.velocity.x = this->odom_.velocity.x;
        msg.velocity.y = this->odom_.velocity.y;
        msg.velocity.theta = this->odom_.velocity.theta;

        msg.velocity_twist = this->odom_.velocity.ToROSTwist();

        msg.detected = this->detected_;
        msg.detection_stamp = this->latest_observed_time_;
        msg.disappeared = this->disappeared_;

        msg.last_detection_pose = this->last_detection_pose.ToROSPose2D();
    }
};

// ObserverFacade クラス
//
// ロボット / ボールらの存在判定、位置・速度推定を実施するFacade
class ObserverFacade
{
public:
    ObserverFacade(int max_id) :
        max_id_(max_id)
    {
        for (auto robot_id=0; robot_id < max_id; ++robot_id)
        {
            blue_observers_.push_back(RobotObserver(robot_id));
            yellow_observers_.push_back(RobotObserver(robot_id));
        }
    }

    void update(ObservationContainer observation_container)
    {
        for (auto robot_id=0; robot_id < this->max_id_; ++robot_id)
        {
            this->blue_observers_[robot_id].update(observation_container.blue_observations[robot_id]);
            this->yellow_observers_[robot_id].update(observation_container.yellow_observations[robot_id]);
        }
        this->ball_observer_.update(observation_container.ball_observations);
    }

private:
    std::vector<RobotObserver> blue_observers_;
    std::vector<RobotObserver> yellow_observers_;
    BallObserver ball_observer_;
    int max_id_;
};

ObserverFacade* observer_facade;

void UpdateHook(ObservationContainer observation_container)
{
    ROS_INFO("hook function called!");

    observer_facade->update(observation_container);
}


int main(int argc, char **argv)
{
    const std::string node_name = "observer";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    // 起動時のnamespaceを取得
    std::string ai_name = "/";
    ros::param::get("ai_name", ai_name);

    std::string vision_topic_name;
    ros::param::param<std::string>("~vision_topic_name", vision_topic_name, "vision_receiver/raw_vision_detections");


    WorldObserverROS    ros_if(nh, vision_topic_name);
    ros_if.RegisterUpdateHook(UpdateHook);

    observer_facade = new ObserverFacade(ros_if.max_id);

    ros::spin();

    return 0;

}
