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


class PoseKalmanFilter
{
    public:
        PoseKalmanFilter();
        ~PoseKalmanFilter();

        void Init(double loop_time);

        geometry2d::Odometry estimate();
        geometry2d::Odometry estimate(std::vector<geometry2d::Pose> observations);
        geometry2d::Odometry estimate(geometry2d::Accel accel, std::vector<geometry2d::Pose> observations);
        void Reset();

    protected:
        double  dt;
        LinearAnalyticConditionalGaussian* sys_pdf;
        LinearAnalyticSystemModelGaussianUncertainty* sys_model;
        LinearAnalyticConditionalGaussian* meas_pdf;
        LinearAnalyticMeasurementModelGaussianUncertainty* meas_model;
        KalmanFilter* filter;
        Gaussian* prior;

        virtual void InitSystemModel(LinearAnalyticConditionalGaussian** sys_pdf, LinearAnalyticSystemModelGaussianUncertainty** sys_model) = 0;
        virtual void InitMeasurementModel(LinearAnalyticConditionalGaussian** meas_pdf, LinearAnalyticMeasurementModelGaussianUncertainty** meas_model)= 0;
        virtual void InitPrior(Gaussian** prior) = 0;
    
    private:
        class Estimation {
            public:
                ColumnVector  val;
                SymmetricMatrix cov;
        };

        Estimation  last_estimation;

        void  predict(ColumnVector input);
        void  update(ColumnVector measurement);

        Estimation getResult();
        void collectAngleOverflow(ColumnVector& state, SymmetricMatrix& cov);
        geometry2d::Odometry convetEstimationToOdometry();

        bool isOutlier(ColumnVector measurement);
        double mahalanobisDistance(ColumnVector measurement);

};


class EnemyEstimator : public PoseKalmanFilter
{
    protected:
        void InitSystemModel(LinearAnalyticConditionalGaussian** sys_pdf, LinearAnalyticSystemModelGaussianUncertainty** sys_model);
        void InitMeasurementModel(LinearAnalyticConditionalGaussian** meas_pdf, LinearAnalyticMeasurementModelGaussianUncertainty** meas_model);
        void InitPrior(Gaussian** prior);
};



class EulerAngle
{
    public:
        static double  normalize(double angle);
        static double  normalize(double angle, double center);
};

#endif
