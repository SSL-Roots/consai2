#include <ros/ros.h>
#include <sstream>
#include <cmath>

#include <world_observer/world_observer_ros.hpp>
#include <world_observer/estimator.hpp>
#include <world_observer/enemy_estimator.hpp>
#include <world_observer/ball_estimator.hpp>

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

    AppearanceMonitor() :
        APPEAR_THRESHOLD_TIME_(0.5),
        DISAPPEAR_THRESHOLD_TIME_(3.0),
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
    ObserverBase()
    {
        this->estimator_.Init(0.016);
    }

    ObserverBase (const ObserverBase& obj)
    {
    }

    void update()
    {
        this->appearance_monitor_.update(false);
        this->detected_ = false;
        if (!(this->appearance_monitor_.is_appear_))
        {
            this->estimator_.Reset();
        }
        else
        {
            this->odom_ = this->estimator_.estimate();
        }
    }

    void update(std::vector<geometry2d::Pose> observations)
    {
        if (observations.size() == 0)
        {
            this->update();
            return;
        }

        this->appearance_monitor_.update(true);
        this->detected_ = true;

        if (!(this->appearance_monitor_.is_appear_))
        {
            this->estimator_.Reset();
        }
        else
        {
            this->odom_ = this->estimator_.estimate(observations);
        }

        // consai2_msgs/RobotInfoへの変換用に保存
        this->last_detection_pose = observations[0];
    }


protected:
    EnemyEstimator estimator_;
    AppearanceMonitor appearance_monitor_;
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
        robot_id_(robot_id)
    {}

    RobotObserver(const RobotObserver& obj) :
        robot_id_(obj.robot_id_)
    {}

    RobotInfo GetInfo()
    {
        RobotInfo info(this->robot_id_, this->odom_, this->detected_, !(this->appearance_monitor_.is_appear_), this->last_detection_pose, this->appearance_monitor_.latest_appeared_time_);

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
    BallInfo GetInfo()
    {
        BallInfo info(this->odom_, this->detected_, !(this->appearance_monitor_.is_appear_), this->last_detection_pose, this->appearance_monitor_.latest_appeared_time_);
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
            blue_observers_.push_back(RobotObserver(robot_id));
            yellow_observers_.push_back(RobotObserver(robot_id));
        }
    }

    void update(ObservationContainer observation_container)
    {
        for (auto robot_id=0; robot_id <= this->max_id_; ++robot_id)
        {
            this->blue_observers_[robot_id].update(observation_container.blue_observations[robot_id]);
            this->yellow_observers_[robot_id].update(observation_container.yellow_observations[robot_id]);
        }
        this->ball_observer_.update(observation_container.ball_observations);
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
