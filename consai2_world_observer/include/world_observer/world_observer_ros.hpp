#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include <consai2_msgs/VisionDetections.h>

#include <world_observer/geometry/geometry.hpp>

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

    WorldObserverROS(ros::NodeHandle& nh, std::string vision_topic_name);
    void VisionCallBack(const consai2_msgs::VisionDetections::ConstPtr& msg);
    bool RegisterUpdateHook(std::function<void(ObservationContainer observation_container)> function);

    void PublishDebugOdom(geometry2d::Odometry odom);

private:
    ros::Subscriber sub_vision_;
    ros::Publisher  pub_odom_debug_;
    std::function<void(ObservationContainer observation_container)> update_hook_;
};



void VisionCallbackHook(const WorldObserverROS& world_observer);
