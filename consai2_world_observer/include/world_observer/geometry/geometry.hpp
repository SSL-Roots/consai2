#ifndef _GEOMETRY_HPP_
#define _GEOMETRY_HPP_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

#include <bfl/filter/extendedkalmanfilter.h>

namespace geometry2d
{

// Poseクラス
// 2次元の姿勢(x, y, theta) を表現します。
class Pose
{
public:
    double x, y, theta;

    Pose(geometry_msgs::Pose   pose);
    Pose(geometry_msgs::Pose2D pose);

    MatrixWrapper::ColumnVector ToColumnVector();
};

class Point
{
public:
    double x, y;
};

double YawFromQuaternion(double x, double y, double z, double w);
double pi2pi(double rad);

}

#endif //_GEOMETRY_HPP_