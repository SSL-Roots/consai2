# consai2_world_observer

このパッケージは、ssl-visionまたはその他のセンサーを入力として、ロボットやボールの位置・速度、およびその他の情報を推定します。

このパッケージは以下の役割を担います。

- (TODO)フィールドサイドの抽象化
    相手ゴールが正、味方ゴールが負となるようにフィールド座標系を回転します。
- 味方・敵情報の抽象化
    チームカラー情報を 相手/味方 という情報に変換します。
- 位置・速度の推定
    Visionからの情報および味方ロボットへの指令値をもとに、ロボットの位置・速度を推定します。
    また、ボールの位置・速度の推定も行います。    

## Subscribe topics

- vision_receiver/raw_vision_detections (consai2_msgs/VisionDetections)

## Publish topics 

- ~ball_info (consai2_msgs/BallInfo
    現在のボールの位置・速度の情報です。)

- ~robot_info_ours_* (consai2_msgs/RobotInfo)
    味方ロボットの位置・速度の情報です。 
    `*` がロボットIDです。

- ~robot_info_theirs_* (consai2_msgs/RobotInfo)
    味方ロボットの位置・速度の情報です。 
    `*` がロボットIDです。

- ~robot_info_blue_* (consai2_msgs/RobotInfo)
    このトピックは後方互換性のために残されていますが、使用は推奨されません。
    味方ロボットへの制御入力が推定に反映されないため、`~robot_info_ours` と比べて情報の信頼度が落ちる可能性があります。

- ~robot_info_yellow_* (consai2_msgs/RobotInfo)
    このトピックは後方互換性のために残されていますが、使用は推奨されません。
    味方ロボットへの制御入力が推定に反映されないため、`~robot_info_ours` と比べて情報の信頼度が落ちる可能性があります。



---
## 開発メモ

### Modules and classes

- No Namspace
    - WorldObserver
        メインのクラス。

        - WorldObserverROS ros_if
        - vector<Estimator> 

- RobotInfo
    ロボットの各種情報を保持するクラス　ファサード的な役割
    推定した座標の情報や、最新の観測情報を持つ

    - Geometry::Pose pose
    - Estimator::PoseKalmanFilter kalman_filter
    - 
        

- Geometry2D
    2次元での座標表現を管理するモジュール
    - Geometry::AbstractGeometry
        2次元の座標系を表現するための抽象クラス

    - Geometry::Point()
        座標 (x, y)を表現するクラス
        - x
        - y
        - toMatrix()

    - Geometry::Pose()
        2次元の座標および回転角を表現するクラス
        - double: x
        - double: y
        - double: theta 
        - transpose(Geometry::Point from)
        - toMatrix()

    - Geometry::Velocity()
        速度を表現するクラス
        - double x
        - double y
        - double theta 
    
    - Geometry::Accel()
        加速度を表現するクラス
        - double x
        - double y
        - double theta 


- WorldObserverROS
    - WorldObserverROS::WorldObserverROS()
        ROSとのIFクラス
        ROSメッセージと Geometryクラスのなかだちをする
        また、SubscriberとPublisherのインスタンスも持つ
        ここで、各ロボットの情報に分割するようにする

        - vector<WorldObserverROS::RobotInfo> blue_robot_infomations
        - vector<WorldObserverROS::RobotInfo> yellow_robot_infomations

    - WorldObserverROS::RobotInfo()
        ロボットの観測情報をまとめるクラス
        - Geometry::Pose2D detections


- WorldAbstraction
    観測情報を抽象化するモジュール

    - TeamAbstractor()
    - GeometryAbstractor()


- Estimator
    カルマンフィルタなどのフィルタを実装するモジュール

    - Estimator::Estimator()
        カルマンフィルタなどの推定器を実装するためのクラス

    - Estimator::PointKalmanFilter() : Estimator::Estimator
        2次元の点に関するカルマンフィルタのクラス

    - Estimator::PoseKalmanFilter() : Estimator::Estimator
        2次元の姿勢に関するカルマンフィルタのクラス
        - update(vector<Geometry::Pose> observation);
        - update(vector<Geometry::Pose> observation, Geometry::);
