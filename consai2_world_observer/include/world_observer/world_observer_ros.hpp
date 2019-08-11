#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <consai2_msgs/VisionDetections.h>


class WorldObserverROS
{
    public:
        std::vector<geometry_msgs::Pose2D> blue_observations;
        std::vector<geometry_msgs::Pose2D> yellow_observations;
        std::vector<geometry_msgs::Pose2D> ball_observations;

        WorldObserverROS(ros::NodeHandle& nh, std::string vision_topic_name);
        void VisionCallBack(const consai2_msgs::VisionDetections::ConstPtr& msg);
        bool RegisterUpdateHook(std::function<void(const WorldObserverROS* world_observer)> function);

    private:
        ros::Subscriber sub_vision_;
        std::function<void(const WorldObserverROS* world_observer)> update_hook_;
};


void VisionCallbackHook(const WorldObserverROS& world_observer);
