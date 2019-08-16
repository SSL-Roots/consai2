#include <cmath>

#include <world_observer/geometry/geometry.hpp>

#include <tf/transform_datatypes.h>


namespace geometry2d
{

Pose::Pose()
{
    this->x = 0.0;
    this->y = 0.0;
    this->theta = 0.0;
}

Pose::Pose(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

Pose::Pose(geometry_msgs::Pose   pose)
{
    this->x = pose.position.x;
    this->y = pose.position.y;
    this->theta = YawFromQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    this->theta = pi2pi(this->theta);
}

Pose::Pose(geometry_msgs::Pose2D pose)
{
    this->x = pose.x;
    this->y = pose.y;
    this->theta = pose.theta;
    this->theta = pi2pi(this->theta);
}

MatrixWrapper::ColumnVector Pose::ToColumnVector()
{
    MatrixWrapper::ColumnVector  column_vector(3);

    column_vector(1) = this->x;
    column_vector(2) = this->y;
    column_vector(3) = this->theta;

    return  column_vector;
}

geometry_msgs::Pose Pose::ToROSPose()
{
    geometry_msgs::Pose msg;

    msg.position.x = this->x;
    msg.position.y = this->y;
    msg.orientation = QuaternionFromYaw(this->theta);

    return msg;
}

geometry_msgs::Pose2D Pose::ToROSPose2D()
{
    geometry_msgs::Pose2D msg;

    msg.x = this->x;
    msg.y = this->y;
    msg.theta = this->theta;

    return msg;
}


// geometry2d::Pose Pose::Transpose(geometry2d::Pose pose)
// このインスタンスの姿勢からみた pose の姿勢を返す
geometry2d::Pose Pose::Transpose(geometry2d::Pose pose)
{
    geometry2d::Pose retval;

    double diff_x = pose.x - this->x;
    double diff_y = pose.y = this->y;

    retval.x = cos(this->theta) * diff_x - sin(this->theta) * diff_y;
    retval.y = sin(this->theta) * diff_x + cos(this->theta) * diff_y;
    retval.theta = pose.theta - this->theta;

    return retval;
}

// double Pose::GetNorm()
// 位置のノルム（距離）を返す
double Pose::GetNorm()
{
    return sqrt(pow(this->x, 2) + pow(this->y, 2));
}

// double Pose::GetAngle()
// 原点からみた自身のなす角を返す（極座標表現での角度）
double Pose::GetAngle()
{
    return atan2(this->y, this->x);
}


Velocity::Velocity()
{
    this->x = 0.0;
    this->y = 0.0;
    this->theta = 0.0;
}

Velocity::Velocity(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}


Velocity::Velocity(geometry_msgs::Twist twist)
{
    this->x = twist.linear.x;
    this->y = twist.linear.y;
    this->theta = twist.angular.z;
}

geometry_msgs::Twist Velocity::ToROSTwist()
{
    geometry_msgs::Twist msg;

    msg.linear.x = this->x;
    msg.linear.y = this->y;
    msg.angular.z = this->theta;

    return msg;
}

// double Velocity::GetNorm()
// 速度ベクトルのノルムを返す
double Velocity::GetNorm()
{
    return sqrt(pow(this->x, 2) + pow(this->y, 2));
}

// double Velocity::GetAngle()
// 速度ベクトルが原点となす角度を返す
double Velocity::GetAngle()
{
    return atan2(this->y, this->x);
}


Accel::Accel()
{
    this->x = 0.0;
    this->y = 0.0;
    this->theta = 0.0;
}

Accel::Accel(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}


Accel::Accel(geometry_msgs::Accel accel)
{
    this->x = accel.linear.x;
    this->y = accel.linear.y;
    this->theta = accel.angular.z;
}

MatrixWrapper::ColumnVector Accel::ToColumnVector()
{
    MatrixWrapper::ColumnVector  column_vector(3);

    column_vector(1) = this->x;
    column_vector(2) = this->y;
    column_vector(3) = this->theta;

    return  column_vector;
}

Odometry::Odometry()
{
}

Odometry::Odometry(Pose pose, Velocity velocity)
{
    this->pose = pose;
    this->velocity = velocity;
}

nav_msgs::Odometry Odometry::ToROSOdometry()
{
    nav_msgs::Odometry msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";    // TODO: パラメータ化

    msg.pose.pose = this->pose.ToROSPose();
    msg.twist.twist = this->velocity.ToROSTwist();

    return msg;
}


void Odometry::print()
{
    std::cout << "[Odometry print]" << std::endl;
    std::cout << "[Pose] x:" << this->pose.x << ", y:" << this->pose.y << ", theta:" << this->pose.theta << std::endl;
    std::cout << "[Velo] x:" << this->velocity.x << ", y:" << this->velocity.y << ", theta:" << this->velocity.theta << std::endl;
}

double YawFromQuaternion(double x, double y, double z, double w)
{
    double  roll, pitch, yaw;
    tf::Quaternion  q(x, y, z, w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::Quaternion QuaternionFromYaw(double theta)
{
    return tf::createQuaternionMsgFromYaw(theta);
}

double pi2pi(double rad)
{
    while(rad >=  M_PI) rad -= 2.0*M_PI;
    while(rad <= -M_PI) rad += 2.0*M_PI;
    return rad;
}

}