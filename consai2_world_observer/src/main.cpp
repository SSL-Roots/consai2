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
    ExistanceChecker(int max_id)
    {
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
        for (auto robot_id=0; robot_id<observation_container.blue_observations.size(); ++robot_id)
        {
            if (observation_container.blue_observations[robot_id].size() > 0)
            {
                this->blue_latest_appeared_time_[robot_id] = now;
            }
        }

        // yellowの観測を取得して、最終出現時刻を更新
        for (auto robot_id=0; robot_id<observation_container.yellow_observations.size(); ++robot_id)
        {
            if (observation_container.yellow_observations[robot_id].size() > 0)
            {
                this->yellow_latest_appeared_time_[robot_id] = now;
            }
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


// ObserverFacade クラス
//
// ロボット / ボールらの存在判定、位置・速度推定を実施するFacade
class ObserverFacade
{
public:
    ObserverFacade(int max_id) :
        existance_checker_(max_id),
        ball_estimator_(0.016)
    {
        this->blue_estimators_.assign(max_id, EnemyEstimator(0.016));
        this->yellow_estimators_.assign(max_id, EnemyEstimator(0.016));
    }

    void update(ObservationContainer observation_container)
    {
        this->existance_checker_.update(observation_container);

        // Blue robots
        for (auto robot_id=0; robot_id < observation_container.blue_observations.size(); ++robot_id)
        {
            geometry2d::Odometry odom;
            
            // if (!this->existance_checker_.IsBlueRobotExist(robot_id))
            // {
            //     this->blue_estimators_[robot_id].Reset();
            //     continue;
            // }

            // if (observation_container.blue_observations[robot_id].size() == 0)
            // {
            //     // 観測なし
            //     odom = this->blue_estimators_[robot_id].estimate();
            // }
            // else
            // {
            //     odom = this->blue_estimators_[robot_id].estimate(observation_container.blue_observations[robot_id]);
            // }

            odom.print();
        }

        // // Yellow robots
        // for (auto robot_id=0; robot_id < observation_container.yellow_observations.size(); ++robot_id)
        // {
        //     if (!this->existance_checker_.IsYellowRobotExist(robot_id))
        //     {
        //         this->yellow_estimators_[robot_id].Reset();
        //         continue;
        //     }

        //     if (observation_container.yellow_observations[robot_id].size() == 0)
        //     {
        //         // 観測なし
        //         this->yellow_estimators_[robot_id].estimate();
        //     }
        //     else
        //     {
        //         this->yellow_estimators_[robot_id].estimate(observation_container.yellow_observations[robot_id]);
        //     }
        // }

        // // balls
        // if (!this->existance_checker_.IsBallExist())
        // {
        //     this->ball_estimator_.Reset();
        // }
        // else
        // {
        //     if (observation_container.ball_observations.size() == 0)
        //     {
        //         // 観測なし
        //         this->ball_estimator_.estimate();
        //     }
        //     else
        //     {
        //         this->ball_estimator_.estimate(observation_container.ball_observations);
        //     }
        // }

        // ROS_INFO("blue_0?: %d", this->existance_checker_.IsYellowRobotExist(0));
    }

    

private:
    ExistanceChecker existance_checker_;

    std::vector<EnemyEstimator> blue_estimators_;
    std::vector<EnemyEstimator> yellow_estimators_;
    EnemyEstimator ball_estimator_;
};


ObserverFacade* observer_facade;

void UpdateHook(ObservationContainer observation_container)
{
    ROS_INFO("hook function called!");

    observer_facade->update(observation_container);
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

    observer_facade = new ObserverFacade(ros_if.max_id);

    ros::spin();

    return 0;

}
