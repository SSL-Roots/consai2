#include <ros/ros.h>
#include <sstream>
#include <cmath>

#include <world_observer/world_observer_ros.hpp>
#include <world_observer/estimator.hpp>

// AppearanceMonitor　クラス
// ロボットまたはボールの出現、および消失をモニタリングするクラス
class AppearanceMonitor
{
public:
    ros::Duration   APPEAR_THRESHOLD_TIME_;
    ros::Duration  DISAPPEAR_THRESHOLD_TIME_;
    ros::Time latest_appeared_time_;
    ros::Time latest_disappeared_time_;
    bool is_appear_;

    AppearanceMonitor(double disappear_threshold_time_sec) :
        APPEAR_THRESHOLD_TIME_(0.05),
        DISAPPEAR_THRESHOLD_TIME_(disappear_threshold_time_sec),
        latest_appeared_time_(ros::Time(0)),
        latest_disappeared_time_(ros::Time(0)),
        is_appear_(false)
    {}


    void update(bool is_observed)
    {
        if (is_observed)
        {
            this->latest_appeared_time_ = ros::Time::now();
        }
        else
        {
            this->latest_disappeared_time_ = ros::Time::now();
        }

        if (this->is_appear_)
        {
            if ((this->latest_disappeared_time_ - this->latest_appeared_time_) > this->DISAPPEAR_THRESHOLD_TIME_)
            {
                this->is_appear_ = false;
            }
        }
        else
        {
            if ((this->latest_appeared_time_ - this->latest_disappeared_time_) > this->APPEAR_THRESHOLD_TIME_)
            {
                this->is_appear_ = true;
            }
        }
    }
};

// ObserverBase クラス
// ロボットまたはボールの位置および速度推定・存在判定を担うクラスのベースクラス
class ObserverBase
{
public:
    ObserverBase(PoseKalmanFilter* p_estimator, AppearanceMonitor* p_appearance_monitor) : 
        p_estimator_(p_estimator),
        p_appearance_monitor_(p_appearance_monitor)
    {
        this->p_estimator_->Init(0.016);
    }

    ~ObserverBase()
    {
        delete this->p_estimator_;
        delete this->p_appearance_monitor_;
    }

    void update()
    {
        this->p_appearance_monitor_->update(false);
        this->detected_ = false;
        if (!(this->p_appearance_monitor_->is_appear_))
        {
            this->p_estimator_->Reset();
        }
        else
        {
            this->odom_ = this->p_estimator_->estimate();
        }
    }

    void update(std::vector<geometry2d::Pose> observations)
    {
        if (observations.size() == 0)
        {
            this->update();
            return;
        }

        this->p_appearance_monitor_->update(true);
        this->detected_ = true;

        if (!(this->p_appearance_monitor_->is_appear_))
        {
            this->p_estimator_->Reset();
        }
        else
        {
            this->odom_ = this->p_estimator_->estimate(observations);
        }

        // consai2_msgs/RobotInfoへの変換用に保存
        this->last_detection_pose = observations[0];
    }

    geometry2d::Odometry GetOdometry()
    {
        return odom_;
    }

    bool IsAppear()
    {
        return this->p_appearance_monitor_->is_appear_;
    }


protected:
    PoseKalmanFilter* p_estimator_;
    AppearanceMonitor* p_appearance_monitor_;
    geometry2d::Odometry odom_;

    bool detected_;
    geometry2d::Pose last_detection_pose;

};

// RobotObserver クラス
//
// ロボット単体の位置及び速度推定・存在判定を担うクラス
class RobotObserver : public ObserverBase
{
public:
    RobotObserver(int robot_id) :
        ObserverBase(new EnemyEstimator(), new AppearanceMonitor(3.0)),
        robot_id_(robot_id)
    {}

    // FIXME: コピーコンストラクタで、新しいBallEstimatorを生成している
    // 本当は同じ内容を持つ別のBallEstimatorを生成すべきな気がする
    RobotObserver(const RobotObserver& obj) :
        ObserverBase(new EnemyEstimator(), new AppearanceMonitor(3.0)),
        robot_id_(obj.robot_id_)
    {}

    RobotInfo GetInfo()
    {
        RobotInfo info(this->robot_id_, this->odom_, this->detected_, !(this->p_appearance_monitor_->is_appear_), this->last_detection_pose, this->p_appearance_monitor_->latest_appeared_time_);

        return info;
    }

private:
    int robot_id_;
};

// BallObserver クラス
//
// ボール単体の位置及び速度推定・存在判定を担うクラス
class BallObserver : public ObserverBase
{
public:
    BallObserver() : 
        ObserverBase(new BallEstimator(), new AppearanceMonitor(1.0))
    {}

    // FIXME: コピーコンストラクタで、新しいBallEstimatorを生成している
    // 本当は同じ内容を持つ別のBallEstimatorを生成すべきな気がする
    BallObserver(const BallObserver& obj) :
        ObserverBase(new BallEstimator(), new AppearanceMonitor(1.0))
    {}

    void updateWithConsideringRobot(std::vector<geometry2d::Odometry> robot_odoms)
    {
        this->p_appearance_monitor_->update(false);
        this->detected_ = false;
        if (!(this->p_appearance_monitor_->is_appear_))
        {
            this->p_estimator_->Reset();
        }
        else
        {
            this->odom_ = this->p_estimator_->estimateWithConsideringOtherRobots(robot_odoms);
        }
    }

    void updateWithConsideringRobot(std::vector<geometry2d::Pose> observations, std::vector<geometry2d::Odometry> robot_odoms)
    {
        if (observations.size() == 0)
        {
            this->update();
            return;
        }

        this->p_appearance_monitor_->update(true);
        this->detected_ = true;

        if (!(this->p_appearance_monitor_->is_appear_))
        {
            this->p_estimator_->Reset();
        }
        else
        {
            this->odom_ = this->p_estimator_->estimateWithConsideringOtherRobots(observations, robot_odoms);
        }

        // consai2_msgs/RobotInfoへの変換用に保存
        this->last_detection_pose = observations[0];
    }

    BallInfo GetInfo()
    {
        BallInfo info(this->odom_, this->detected_, !(this->p_appearance_monitor_->is_appear_), this->last_detection_pose, this->p_appearance_monitor_->latest_appeared_time_);
        return info;
    }
};

// ObserverFacade クラス
//
// ロボット / ボールらの存在判定、位置・速度推定を実施するFacade
class ObserverFacade
{
public:
    int max_id_;

    ObserverFacade(int max_id) :
        max_id_(max_id)
    {
        for (auto robot_id=0; robot_id <= max_id; ++robot_id)
        {
            this->blue_observers_.push_back(RobotObserver(robot_id));
            this->yellow_observers_.push_back(RobotObserver(robot_id));
        }
    }

    void update(ObservationContainer observation_container)
    {
        std::vector<geometry2d::Odometry> all_robots_odom;
        all_robots_odom.reserve(this->blue_observers_.size() + this->yellow_observers_.size());

        for (auto robot_id=0; robot_id <= this->max_id_; ++robot_id)
        {
            this->blue_observers_[robot_id].update(observation_container.blue_observations[robot_id]);
            if (this->blue_observers_[robot_id].IsAppear())
            {
                all_robots_odom.push_back(this->blue_observers_[robot_id].GetOdometry());
            }

            this->yellow_observers_[robot_id].update(observation_container.yellow_observations[robot_id]);
            if (this->yellow_observers_[robot_id].IsAppear())
            {
                all_robots_odom.push_back(this->yellow_observers_[robot_id].GetOdometry());
            }
        }
        this->ball_observer_.updateWithConsideringRobot(observation_container.ball_observations, all_robots_odom);
    }

    RobotInfo GetBlueInfo(int robot_id)
    {
        return this->blue_observers_[robot_id].GetInfo();
    }

    RobotInfo GetYellowInfo(int robot_id)
    {
        return this->yellow_observers_[robot_id].GetInfo();
    }

    BallInfo GetBallInfo()
    {
        return this->ball_observer_.GetInfo();
    }

    

private:
    std::vector<RobotObserver> blue_observers_;
    std::vector<RobotObserver> yellow_observers_;
    BallObserver ball_observer_;
};

ObserverFacade* observer_facade;
std::unique_ptr<WorldObserverROS> world_observer_ros;

void UpdateHook(ObservationContainer observation_container)
{
    observer_facade->update(observation_container);
    BallInfo info = observer_facade->GetBallInfo();

    world_observer_ros->PublishBallInfo(info);

    for (auto robot_id=0; robot_id<=observer_facade->max_id_; ++robot_id)
    {
        RobotInfo blue_info = observer_facade->GetBlueInfo(robot_id);
        world_observer_ros->PublishBlueInfo(robot_id, blue_info);

        RobotInfo yellow_info = observer_facade->GetYellowInfo(robot_id);
        world_observer_ros->PublishYellowInfo(robot_id, yellow_info);
    }
}


int main(int argc, char **argv)
{
    const std::string node_name = "consai2_world_observer";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // 起動時のnamespaceを取得
    std::string ai_name = "/";
    ros::param::get("ai_name", ai_name);

    std::string vision_topic_name;
    ros::param::param<std::string>("~vision_topic_name", vision_topic_name, "vision_receiver/raw_vision_detections");

    world_observer_ros.reset(new WorldObserverROS(nh, nh_private, vision_topic_name));
    world_observer_ros->RegisterUpdateHook(UpdateHook);

    observer_facade = new ObserverFacade(world_observer_ros->max_id);

    ros::spin();

    return 0;

}
