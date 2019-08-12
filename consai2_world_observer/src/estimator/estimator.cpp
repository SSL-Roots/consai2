#include  <iostream>

#include <world_observer/estimator.hpp>



Estimator::Estimator()
{}

geometry2d::Odometry Estimator::estimate()
{
    std::vector<geometry2d::Pose> null_poses;
    return  this->estimate(null_poses);
}

geometry2d::Odometry Estimator::estimate(std::vector<geometry2d::Pose> observations)
{
    geometry2d::Accel  null_acc;
    return  this->estimate(null_acc, observations);
}

geometry2d::Odometry Estimator::estimate(geometry2d::Accel accel, std::vector<geometry2d::Pose> observations)
{
    // System update by only system model with input
    predict(accel.ToColumnVector());

    for (auto observation : observations)
    {
        ROS_INFO("observation x:%3.2f, y:%3.2f", observation.x, observation.y);
        MatrixWrapper::ColumnVector observation_cv = observation.ToColumnVector();

        // if (isOutlier(observation_cv)) {
        //     continue;
        // }

        update(observation_cv);
    }

    return convetEstimationToOdometry();
}


void Estimator::Reset()
{
    this->filter->Reset(this->prior);
}

//
// Private methods
//


Estimator::~Estimator()
{
}
