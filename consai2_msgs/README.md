# consai2_msgs

consai2で使うtopicやserviceを生成してます

## Topics
- 
- BallInfo
  - DetectionBallをもとに生成されたボールの位置・速度・消失情報
- ControlTarget
  - ロボットの制御目標値
- DetectionBall
  - visionから得られるボールの位置
- DetectionFrame
  - visionから得られるフレーム
- DetectionRobot
  - visionから得られるロボット位置・ID
- FieldCircularArc
  - visionから得られるフィールドの曲線情報
- FieldLineSegment
  - visionから得られるフィールドの直線情報
- Referee
  - refereeから得られるレフェリー信号
- RefereeGameEvent
  - refereeから得られるイベント情報
- RefereeTeamInfo
  - refereeから得られるチーム情報
- ReplaceBall
  - grSimへ送るボール再配置位置
- ReplaceRobot
  - grSimへ送るロボット再配置位置
- Replacements
  - grSimへ再配置情報
- RobotCommand
  - ロボットへの制御命令
- RobotCommands
  - チームカラー情報を付加したRobotCommandのリスト
- RobotInfo
  - DetectionRobotをもとに生成されたロボットの位置・速度・消失情報
- VisionDetections
  - visionから得られるフレームのリスト
- VisionGeometry
  - visionから得られるフィールド線分のリスト
