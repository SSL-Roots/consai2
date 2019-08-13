#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include <consai2_msgs/VisionDetections.h>
#include <consai2_msgs/RobotInfo.h>
#include <consai2_msgs/BallInfo.h>

#include <world_observer/geometry/geometry.hpp>

// InfoBaseクラス
// consai2_msgs/RobotInfo　及び consai2_msgs/BallInfoに対応するクラスのベースクラス
class InfoBase
{
public:
    InfoBase(geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp);

    geometry2d::Odometry odom_;
    bool detected_;
    bool disappeared_;
    geometry2d::Pose last_detection_pose_;
    ros::Time detection_stamp_;
};

// RobotInfoクラス
// consai2_msgs/RobotInfoに対応するクラス
class RobotInfo : public InfoBase
{
public:
    RobotInfo(int robot_id, geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp);
    consai2_msgs::RobotInfo ToROSMsg();

private:
    int robot_id_;
};

// BallInfoクラス
// consai2_msgs/BallInfoに対応するクラス
class BallInfo : public InfoBase
{
public:
    BallInfo(geometry2d::Odometry odom, bool detected, bool disappeared, geometry2d::Pose last_detection_pose, ros::Time detection_stamp);
    consai2_msgs::BallInfo ToROSMsg();
};

// ObservationContainer
// 一回のVisionによる観測値を格納しておくクラス
class ObservationContainer
{
public:
    std::vector<std::vector<geometry2d::Pose>> blue_observations;
    std::vector<std::vector<geometry2d::Pose>> yellow_observations;
    std::vector<geometry2d::Pose>                ball_observations;

    ObservationContainer(int num_of_robot);
};

// WorldObserverROS
// ROSとのIFクラス SubsciberやPublisherはここにまとめる
// ここを境目にROSのメッセージ型と内部で扱う型を分離する
class WorldObserverROS
{
public:
    int max_id;

    WorldObserverROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::string vision_topic_name);
    void VisionCallBack(const consai2_msgs::VisionDetections::ConstPtr& msg);
    bool RegisterUpdateHook(std::function<void(ObservationContainer observation_container)> function);

    void PublishDebugOdom(geometry2d::Odometry odom);
    void PublishBallInfo(BallInfo info);

private:
    ros::Subscriber sub_vision_;
    ros::Publisher  pub_ball_info_;
    ros::Publisher  pub_odom_debug_;
    std::function<void(ObservationContainer observation_container)> update_hook_;
};



void VisionCallbackHook(const WorldObserverROS& world_observer);
