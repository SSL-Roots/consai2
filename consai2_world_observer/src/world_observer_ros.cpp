#include <world_observer/world_observer_ros.hpp>

WorldObserverROS::WorldObserverROS(ros::NodeHandle& nh, std::string vision_topic_name) :
    sub_vision_(nh.subscribe(vision_topic_name, 10, &WorldObserverROS::VisionCallBack, this))
{
}

void WorldObserverROS::VisionCallBack(const consai2_msgs::VisionDetections::ConstPtr& msg)
{
    // 前回観測値の削除
    blue_observations.clear();
    yellow_observations.clear();
    ball_observations.clear();

    // 観測値をロボットIDごと、ボールごとに格納
    for (auto frame : msg->frames)
    {
        ROS_INFO("camera_id: %i", frame.camera_id);

        for (auto blue_observation : frame.robots_blue)
        {
            this->blue_observations.push_back(blue_observation.pose);
        }

        for (auto yellow_observation : frame.robots_yellow)
        {
            this->yellow_observations.push_back(yellow_observation.pose);
        }

        for (auto ball_observation : frame.balls)
        {
            this->ball_observations.push_back(ball_observation.pose);
        }
    }

    //フック関数の呼び出し
    if (this->update_hook_) {
        this->update_hook_(this);
    }
}

bool WorldObserverROS::RegisterUpdateHook(std::function<void(const WorldObserverROS* world_observer)> function)
{
    this->update_hook_ = function;
    return true;
}