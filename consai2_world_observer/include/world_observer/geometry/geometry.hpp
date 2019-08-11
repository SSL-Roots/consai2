#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

namespace geometry2d
{

// Poseクラス
// 2次元の姿勢(x, y, theta) を表現します。
class Pose
{
public:
    Pose(geometry_msgs::Pose   pose);
    Pose(geometry_msgs::Pose2D pose);
    double x, y, theta;
};

class Point
{
public:
    double x, y;
};

double YawFromQuaternion(double x, double y, double z, double w);

}