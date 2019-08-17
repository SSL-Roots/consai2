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

        virtual geometry2d::Odometry estimateWithConsideringOtherRobots(std::vector<geometry2d::Odometry> other_robots) = 0;
        virtual geometry2d::Odometry estimateWithConsideringOtherRobots(std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots) = 0;
        virtual geometry2d::Odometry estimateWithConsideringOtherRobots(geometry2d::Accel accel, std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots) = 0;

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
        geometry2d::Odometry convetEstimationToOdometry();
    
    private:
        class Estimation {
            public:
                ColumnVector  val;
                SymmetricMatrix cov;
        };

        ros::Duration KIDNAPPED_TIME_THRESH_;
        Estimation  last_estimation;
        ros::Time   latest_inlier_stamp_;

        void  predict(ColumnVector input);
        void  update(ColumnVector measurement);

        Estimation getResult();
        void collectAngleOverflow(ColumnVector& state, SymmetricMatrix& cov);

        bool isOutlier(ColumnVector measurement);
        double mahalanobisDistance(ColumnVector measurement);

};


class EnemyEstimator : public PoseKalmanFilter
{
    public:
        geometry2d::Odometry estimateWithConsideringOtherRobots(std::vector<geometry2d::Odometry> other_robots);
        geometry2d::Odometry estimateWithConsideringOtherRobots(std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots);
        geometry2d::Odometry estimateWithConsideringOtherRobots(geometry2d::Accel accel, std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots);

    protected:
        void InitSystemModel(LinearAnalyticConditionalGaussian** sys_pdf, LinearAnalyticSystemModelGaussianUncertainty** sys_model);
        void InitMeasurementModel(LinearAnalyticConditionalGaussian** meas_pdf, LinearAnalyticMeasurementModelGaussianUncertainty** meas_model);
        void InitPrior(Gaussian** prior);
};


class BallEstimator : public PoseKalmanFilter
{
    public:
        geometry2d::Odometry estimateWithConsideringOtherRobots(std::vector<geometry2d::Odometry> other_robots);
        geometry2d::Odometry estimateWithConsideringOtherRobots(std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots);
        geometry2d::Odometry estimateWithConsideringOtherRobots(geometry2d::Accel accel, std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> other_robots);

    protected:
        void InitSystemModel(LinearAnalyticConditionalGaussian** sys_pdf, LinearAnalyticSystemModelGaussianUncertainty** sys_model);
        void InitMeasurementModel(LinearAnalyticConditionalGaussian** meas_pdf, LinearAnalyticMeasurementModelGaussianUncertainty** meas_model);
        void InitPrior(Gaussian** prior);

    private:
        // BallReflectionDetector クラス
        // ボールがキックまたは跳ね返ることを検出するクラス
        // 急激な速度変化がある際はカルマンフィルタのシステムノイズを大きく設定し、フィルタの追従性を向上させるため、そのサポート役のクラス
        class BallReflectionDetector
        {
        public:
            bool WillReflectionOccur(geometry2d::Odometry odom_ball, std::vector<geometry2d::Odometry> odom_robots);
        private:
            bool IsBallInFrontOfRobot(geometry2d::Pose pose_robot, geometry2d::Pose pose_ball);
            bool WillBallContactToRobotSoon(geometry2d::Odometry odom_robot, geometry2d::Odometry odom_ball);
        };

        BallReflectionDetector  ball_reflection_detector_;

        void SetSytemNoiseForReflecting();
        void SetSystemNoiseToRolling();
};



class EulerAngle
{
    public:
        static double  normalize(double angle);
        static double  normalize(double angle, double center);
};

#endif
