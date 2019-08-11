#include <world_observer/geometry/geometry.hpp>

#include <tf/transform_datatypes.h>


namespace geometry2d
{

Pose::Pose(geometry_msgs::Pose   pose)
{
    this->x = pose.position.x;
    this->y = pose.position.y;
    this->theta = YawFromQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

Pose::Pose(geometry_msgs::Pose2D pose)
{
    this->x = pose.x;
    this->y = pose.y;
    this->theta = pose.theta;
}

double YawFromQuaternion(double x, double y, double z, double w)
{
    double  roll, pitch, yaw;
    tf::Quaternion  q(x, y, z, w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    return yaw;
}

}