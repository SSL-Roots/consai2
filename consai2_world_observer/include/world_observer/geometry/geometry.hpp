#ifndef _GEOMETRY_HPP_
#define _GEOMETRY_HPP_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

#include <bfl/filter/extendedkalmanfilter.h>

namespace geometry2d
{

//
// 単純型
//

// Poseクラス
// 2次元の姿勢(x, y, theta) を表現します。
class Pose
{
public:
    double x, y, theta;

    Pose();
    Pose(geometry_msgs::Pose   pose);
    Pose(geometry_msgs::Pose2D pose);

    MatrixWrapper::ColumnVector ToColumnVector();
};

class Velocity
{
public:
    double x, y, theta;

    Velocity();
    Velocity(double x, double y, double theta);
    Velocity(geometry_msgs::Twist twist);
};

class Accel
{
public:
    double x, y, theta;

    Accel();
    Accel(double x, double y, double theta);
    Accel(geometry_msgs::Accel accel);

    MatrixWrapper::ColumnVector ToColumnVector();
};

class Velocity
{
public:
    double x, y, theta;

    Velocity();
    Velocity(double x, double y, double theta);
    Velocity(geometry_msgs::Twist twist);
};


class Point
{
public:
    double x, y;
};

//
// 複合型
//
class Odometry
{
public:
    Pose pose;
    Velocity velocity;

    Odometry();
    Odometry(Pose pose, Velocity velocity);
};

// Utility methods
double YawFromQuaternion(double x, double y, double z, double w);
double pi2pi(double rad);

}

#endif //_GEOMETRY_HPP_