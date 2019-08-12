#ifndef _ESTIMATOR_HPP_
#define _ESTIMATOR_HPP_

#include  "ros/ros.h"
#include  <geometry_msgs/Pose.h>
#include  <geometry_msgs/Accel.h>
#include  <nav_msgs/Odometry.h>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <world_observer/geometry/geometry.hpp>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;


class Estimator
{
    public:
        Estimator();
        ~Estimator();

        geometry2d::Odometry estimate();
        geometry2d::Odometry estimate(std::vector<geometry2d::Pose> observations);
        geometry2d::Odometry estimate(geometry2d::Accel accel, std::vector<geometry2d::Pose> observations);

    protected:
        class Estimation {
            public:
                ColumnVector  val;
                SymmetricMatrix cov;
        };

        Estimation  last_estimation;

        virtual void  predict(ColumnVector input) = 0;
        virtual void  update(ColumnVector measurement) = 0;

        virtual ColumnVector  convertPoseMsgToMeasureVector(geometry_msgs::Pose pose) = 0;
        virtual ColumnVector  convertAccelMsgToInputVector(geometry_msgs::Accel acc) = 0;
        virtual geometry2d::Odometry convetEstimationToOdometry() = 0;

};



#endif
