#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <consai_msgs/VisionPacket.h>
#include <consai_msgs/VisionData.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <sstream>
#include <cmath>

#include <world_observer/world_observer_ros.hpp>
#include <world_observer/estimator.hpp>
#include <world_observer/enemy_estimator.hpp>
#include <world_observer/ball_estimator.hpp>

// ExistanceCheckerクラス
// ロボット,ボールの存在判定を行うクラス
class ExistanceChecker
{
public:
    ExistanceChecker()
    {
        int max_id;
        ros::param::param<int>("consai2_description/max_id", max_id, 15);

        this->max_id_ = max_id;
        this->blue_existance_.assign(max_id, false);
        this->yellow_existance_.assign(max_id, false);
        this->blue_latest_appeared_time_.assign(max_id, ros::Time(0));
        this->yellow_latest_appeared_time_.assign(max_id, ros::Time(0));
        
        this->ball_existance_ = false;
        this->ball_latest_appeared_time_ = ros::Time(0);

        this->disapper_threshold_time_ = ros::Duration(3.0);
    }

    void update(ObservationContainer observation_container)
    {
        ros::Time now = ros::Time::now();

        // blueの観測を取得して、最終出現時刻を更新
        for( auto mp : observation_container.blue_observations)
        {
            int robot_id = mp.first;
            this->blue_latest_appeared_time_[robot_id]  = now;
        }

        // yellowの観測を取得して、最終出現時刻を更新
        for( auto mp : observation_container.yellow_observations)
        {
            int robot_id = mp.first;
            this->yellow_latest_appeared_time_[robot_id]  = now;
        }

        // ballの観測を取得して、最終出現時刻を更新
        if (observation_container.ball_observations.size() > 0)
        {
            this->ball_latest_appeared_time_ = now;
        }

        // 存在を更新
        for (auto i=0; i < this->max_id_; ++i)
        {
            ros::Time   latest_appeared_time = blue_latest_appeared_time_[i];
            if ((now - latest_appeared_time) > this->disapper_threshold_time_ ) 
            {
                blue_existance_[i] = false;
            }
            else
            {
                blue_existance_[i] = true;
            }
        }

        for (auto i=0; i < this->max_id_; ++i)
        {
            ros::Time   latest_appeared_time = yellow_latest_appeared_time_[i];
            if ((now - latest_appeared_time) > this->disapper_threshold_time_ ) 
            {
                yellow_existance_[i] = false;
            }
            else
            {
                yellow_existance_[i] = true;
            }
        }

        ros::Time   latest_appeared_time = ball_latest_appeared_time_;
        if ((now - latest_appeared_time) > this->disapper_threshold_time_ ) 
        {
            ball_existance_ = false;
        }
        else
        {
            ball_existance_ = true;
        }

    }

    bool IsBlueRobotExist(int robot_id)
    {
        if (robot_id > this->max_id_) {
            return false;
        }
        return this->blue_existance_[robot_id];
    }

    bool IsYellowRobotExist(int robot_id)
    {
        if (robot_id > this->max_id_) {
            return false;
        }
        return this->yellow_existance_[robot_id];
    }

    bool IsBallExist()
    {
        return this->ball_existance_;
    }


private:
    ros::Duration   disapper_threshold_time_;
    int max_id_;
    std::vector<bool> blue_existance_;
    std::vector<bool> yellow_existance_;
    bool              ball_existance_;
    std::vector<ros::Time> blue_latest_appeared_time_;
    std::vector<ros::Time> yellow_latest_appeared_time_;
    ros::Time              ball_latest_appeared_time_;
};


// Observer クラス
//
// ロボット / ボール単体の状態推定を行う。
// 位置・速度の推定、存在判定
class Observer
{
public:
    ros::Duration DISAPPEARED_TIME_THRESH;
    ros::Time latest_appeared_time;
    EnemyEstimator estimator;
    bool is_exist;

    Observer() : 
        DISAPPEARED_TIME_THRESH(3.0),
        is_exist(true),
        latest_appeared_time(ros::Time(0)),
        estimator(0.016)
    {
    }

    void update()
    {
        ros::Time now = ros::Time::now();

        if ((now - this->latest_appeared_time) > this->DISAPPEARED_TIME_THRESH)
        {
            this->is_exist = false;
        }

        this->estimator.estimate();
    }

    void update(std::vector<geometry2d::Pose> vision_observations)
    {
        this->is_exist = true;
        this->estimator.estimate(vision_observations);
        this->latest_appeared_time = ros::Time::now();
    }

};



std::map<int, Observer> ours_observers;
std::map<int, Observer> theirs_observers;
Observer ball_observers;
ExistanceChecker existance_checker;

void UpdateHook(ObservationContainer observation_container)
{
    ROS_INFO("hook function called!");

    existance_checker.update(observation_container);
    ROS_INFO("blue_0_exist?: %d", existance_checker.IsBlueRobotExist(0));
    // ROS_INFO("x:%3.2f, y:%3.2f", observation_container.blue_observations[0][0].x, observation_container.blue_observations[0][0].y);
    

    // geometry2d::Odometry odom;
    // odom = enemy_estimator.estimate(observation_container.blue_observations[0]);

    // ROS_INFO("[RAW] x: %3.2f, y:%3.2f", x, y);
    // ROS_INFO("[FIL] x: %3.2f, y:%3.2f", odom.pose.x, odom.pose.y);
    // ROS_INFO("-----");

    // world_observer->PublishDebugOdom(odom);
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

    ros::spin();

    return 0;

}
