#ifndef _ENEMY_ESTIMATOR_HPP_
#define _ENEMY_ESTIMATOR_HPP_

#include  <world_observer/estimator.hpp>

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
