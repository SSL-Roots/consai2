#include <world_observer/world_observer_ros.hpp>

WorldObserverROS::WorldObserverROS(ros::NodeHandle& nh, std::string vision_topic_name) :
    sub_vision_(nh.subscribe(vision_topic_name, 10, &WorldObserverROS::VisionCallBack, this)),
    pub_odom_debug_(nh.advertise<nav_msgs::Odometry>("debug_odom", 1000))
{
}

void WorldObserverROS::VisionCallBack(const consai2_msgs::VisionDetections::ConstPtr& msg)
{
    ObservationContainer observation_container;

    // 観測値をロボットIDごと、ボールごとに格納
    for (auto frame : msg->frames)
    {
        for (auto blue_observation : frame.robots_blue)
        {
            geometry2d::Pose pose(blue_observation.pose);
            observation_container.blue_observations[blue_observation.robot_id].push_back(pose);
        }

        for (auto yellow_observation : frame.robots_yellow)
        {
            geometry2d::Pose pose(yellow_observation.pose);
            observation_container.yellow_observations[yellow_observation.robot_id].push_back(pose);
        }

        for (auto ball_observation : frame.balls)
        {
            geometry2d::Pose pose(ball_observation.pose);
            observation_container.ball_observations.push_back(pose);
        }
    }

    //フック関数の呼び出し
    if (this->update_hook_) {
        this->update_hook_(observation_container);
    }
}

void WorldObserverROS::PublishDebugOdom(geometry2d::Odometry odom)
{
    nav_msgs::Odometry msg;
    msg = odom.ToROSOdometry();

    this->pub_odom_debug_.publish(msg);
}

bool WorldObserverROS::RegisterUpdateHook(std::function<void(ObservationContainer observation_container)> function)
{
    this->update_hook_ = function;
    return true;
}