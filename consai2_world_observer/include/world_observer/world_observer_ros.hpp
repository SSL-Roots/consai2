#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include <consai2_msgs/VisionDetections.h>

#include <world_observer/geometry/geometry.hpp>


class WorldObserverROS
{
    public:
        std::map<int, std::vector<geometry2d::Pose>> blue_observations;
        std::map<int, std::vector<geometry2d::Pose>> yellow_observations;
        std::vector<geometry2d::Pose>                ball_observations;

        WorldObserverROS(ros::NodeHandle& nh, std::string vision_topic_name);
        void VisionCallBack(const consai2_msgs::VisionDetections::ConstPtr& msg);
        bool RegisterUpdateHook(std::function<void(WorldObserverROS* world_observer)> function);

    private:
        ros::Subscriber sub_vision_;
        std::function<void(WorldObserverROS* world_observer)> update_hook_;
};


void VisionCallbackHook(const WorldObserverROS& world_observer);
