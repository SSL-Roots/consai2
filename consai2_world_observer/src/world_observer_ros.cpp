#include <iostream>
#include <sstream>
#include <world_observer/world_observer_ros.hpp>


// InfoBaseクラス
// consai2_msgs/RobotInfo　及び consai2_msgs/BallInfoに対応するクラスのベースクラス
InfoBase::InfoBase(geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp)
{
    this->odom_ = odom;
    this->detected_ = detected;
    this->disappeared_ = disappeared;
    this->last_detection_pose_ = last_detection_pose;
    this->detection_stamp_ = detection_stamp;
}

// RobotInfoクラス
// consai2_msgs/RobotInfoに対応するクラス
RobotInfo::RobotInfo(int robot_id, geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp) :
    InfoBase(odom, detected, disappeared, last_detection_pose, detection_stamp),
    robot_id_(robot_id)
{
}

consai2_msgs::RobotInfo RobotInfo::ToROSMsg()
{
    consai2_msgs::RobotInfo msg;

    msg.robot_id = this->robot_id_;
    msg.pose = this->odom_.pose.ToROSPose2D();

    msg.velocity.x = this->odom_.velocity.x;
    msg.velocity.y = this->odom_.velocity.y;
    msg.velocity.theta = this->odom_.velocity.theta;

    msg.velocity_twist = this->odom_.velocity.ToROSTwist();

    msg.detected = this->detected_;
    msg.detection_stamp = this->detection_stamp_;
    msg.disappeared = this->disappeared_;

    msg.last_detection_pose = this->last_detection_pose_.ToROSPose2D();

    return msg;
}

// BallInfoクラス
// consai2_msgs/BallInfoに対応するクラス
BallInfo::BallInfo(geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp) :
    InfoBase(odom, detected, disappeared, last_detection_pose, detection_stamp)
{
}

consai2_msgs::BallInfo BallInfo::ToROSMsg()
{
    consai2_msgs::BallInfo msg;

    msg.pose = this->odom_.pose.ToROSPose2D();

    msg.velocity.x = this->odom_.velocity.x;
    msg.velocity.y = this->odom_.velocity.y;
    msg.velocity.theta = this->odom_.velocity.theta;

    msg.velocity_twist = this->odom_.velocity.ToROSTwist();

    msg.detected = this->detected_;
    msg.detection_stamp = this->detection_stamp_;
    msg.disappeared = this->disappeared_;

    msg.last_detection_pose = this->last_detection_pose_.ToROSPose2D();

    return msg;
}

//
// ObservationContainer クラス
//
ObservationContainer::ObservationContainer(int num_of_robot)
{
    std::vector<geometry2d::Pose>   null_poses;
    this->blue_observations.assign(num_of_robot, null_poses);
    this->yellow_observations.assign(num_of_robot, null_poses);
}


//
// WorldObserverROS クラス
//
WorldObserverROS::WorldObserverROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::string vision_topic_name) :
    sub_vision_(nh.subscribe(vision_topic_name, 10, &WorldObserverROS::VisionCallBack, this)),
    pub_ball_info_(nh_private.advertise<consai2_msgs::BallInfo>("ball_info", 1000)),
    pub_odom_debug_(nh.advertise<nav_msgs::Odometry>("debug_odom", 1000))
{
    ros::param::param<int>("consai2_description/max_id", this->max_id, 15);

    for (auto robot_id=0; robot_id <= max_id; ++robot_id)
    {
        std::ostringstream topic_name_blue;
        topic_name_blue << "robot_info_blue_" << robot_id;
        this->p_pub_blue_info_.push_back(new ros::Publisher);
        *(this->p_pub_blue_info_.back()) = nh_private.advertise<consai2_msgs::RobotInfo>(topic_name_blue.str(), 1000);

        std::ostringstream topic_name_yellow;
        topic_name_yellow << "robot_info_yellow_" << robot_id;
        this->p_pub_yellow_info_.push_back(new ros::Publisher);
        *(this->p_pub_yellow_info_.back()) = nh_private.advertise<consai2_msgs::RobotInfo>(topic_name_yellow.str(), 1000);
    }
}

void WorldObserverROS::VisionCallBack(const consai2_msgs::VisionDetections::ConstPtr& msg)
{
    ObservationContainer observation_container(this->max_id + 1);

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


void WorldObserverROS::PublishBlueInfo(int robot_id, RobotInfo info)
{
    if (robot_id >= this->p_pub_blue_info_.size())
    {
        return;
    }
    this->p_pub_blue_info_[robot_id]->publish(info.ToROSMsg());
}

void WorldObserverROS::PublishYellowInfo(int robot_id, RobotInfo info)
{
    if (robot_id >= this->p_pub_yellow_info_.size())
    {
        return;
    }
    this->p_pub_yellow_info_[robot_id]->publish(info.ToROSMsg());
}

void WorldObserverROS::PublishBallInfo(BallInfo info)
{
    this->pub_ball_info_.publish(info.ToROSMsg());
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

WorldObserverROS::~WorldObserverROS()
{
    for (auto p : this->p_pub_blue_info_)
    {
        delete p;
    }

    for (auto p : this->p_pub_yellow_info_)
    {
        delete p;
    }
}