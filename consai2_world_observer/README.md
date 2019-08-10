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

- ~ball_info (consai2_msgs/BallInfo)
    現在のボールの位置・速度の情報です。

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

