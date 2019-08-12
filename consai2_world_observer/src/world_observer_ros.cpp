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
            geometry2d::Pose pose(blue_observation.pose);
            this->blue_observations[blue_observation.robot_id].push_back(pose);
        }

        for (auto yellow_observation : frame.robots_yellow)
        {
            geometry2d::Pose pose(yellow_observation.pose);
            this->yellow_observations[yellow_observation.robot_id].push_back(pose);
        }

        for (auto ball_observation : frame.balls)
        {
            geometry2d::Pose pose(ball_observation.pose);
            this->ball_observations.push_back(pose);
        }
    }

    //フック関数の呼び出し
    if (this->update_hook_) {
        this->update_hook_(this);
    }
}

bool WorldObserverROS::RegisterUpdateHook(std::function<void(WorldObserverROS* world_observer)> function)
{
    this->update_hook_ = function;
    return true;
}