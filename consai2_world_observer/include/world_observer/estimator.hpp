#ifndef _ESTIMATOR_HPP_
#define _ESTIMATOR_HPP_

#include  "ros/ros.h"

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
        void Reset();

    protected:
        class Estimation {
            public:
                ColumnVector  val;
                SymmetricMatrix cov;
        };

        LinearAnalyticConditionalGaussian* sys_pdf;
        LinearAnalyticSystemModelGaussianUncertainty* sys_model;
        LinearAnalyticConditionalGaussian* meas_pdf;
        LinearAnalyticMeasurementModelGaussianUncertainty* meas_model;
        KalmanFilter* filter;
        Gaussian* prior;

        Estimation  last_estimation;

        virtual void  predict(ColumnVector input) = 0;
        virtual void  update(ColumnVector measurement) = 0;

        virtual geometry2d::Odometry convetEstimationToOdometry() = 0;

};


class EnemyEstimator : public Estimator
{
    public:
        // constructor
        EnemyEstimator();

        void Init(double loop_time);

        ~EnemyEstimator();

    protected:
        double  dt;

        void initSystemModel();
        void initMeasurementModel();

        void  predict(ColumnVector input);
        void  update(ColumnVector measurement);

        geometry2d::Odometry convetEstimationToOdometry();

        bool isOutlier(ColumnVector measurement);
        double mahalanobisDistance(ColumnVector measurement);

        Estimation getResult();

        void collectAngleOverflow(ColumnVector& state, SymmetricMatrix& cov);
};



class EulerAngle
{
    public:
        static double  normalize(double angle);
        static double  normalize(double angle, double center);
};

#endif
