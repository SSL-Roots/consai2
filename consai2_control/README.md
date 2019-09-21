# consai2_control

ロボットの制御指令を生成するパッケージです。

# Example

[`scripts/example/example_control.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_control/scripts/example/example_control.py)
がサンプルコードです。

## 概要

フィールド上にいる自チームのロボットに対して、
`ControlTarget`に対応した制御指令`RobotCommand`を生成します。

## 経路追従

`ControlTarget.path`がセットされていればその経路を追従します。

## 速度制御

`ControlTarget.path`がセットされていない場合、
`ControlTarget.goal_velocity`を目標にロボット速度を制御します。

速度制御には加速度リミッタと速度リミッタが組み込まれています。

## 位置制御

`ControlTarget.path`がセットされている場合、
その目標位置とロボットの現在位置の差を用いてPID制御器を実行します。

PID制御器には速度型PID制御器を使用しています。

PID制御器から生成されたロボットの目標速度は、さきほどの速度制御器に渡されます。

## キック・ドリブル情報の付加

`ControlTarget`にはキック・ドリブルの信号も含まれています。
その信号を`RobotComand`に変換しています。

## 座標変換

`ControlTarget`の目標経路（位置）`path`と、目標速度`goal_velocity`はフィールドの座標系で表現されています。
`RobotCommand`の制御速度はロボット座標系で表現されています。

制御速度はフィールド座標系で更新した後、ロボット座標系に変換し、`RobotCommand`に格納しています。

### ロボット座標系

ロボット座標系は右手系です。

![robotCoordinate](https://github.com/SSL-Roots/consai2/blob/images/images/robotCoordinate.png)

## Subscribe topics

- `vision_wrapper/robot_info_*` (consai2_msgs/RobotInfo)
- `consai2_game/control_target_*` (consai2_msgs/ControlTarget)

## Publish topics

- `~/is_arrived_*` (std_msgs/Bool)
  - 目標位置到達フラグ
  - ロボットが経路の最終地点に到達したら`True`
- `~/command_velocity_*` (geometry_msgs/Pose2D)
  - フィールド座標系での制御速度
  - 位置情報のフィルタリングや制御のデバッグに使用する
- `~/robot_commands` (consai2_msgs/RobotCommands)
  - ロボットへの動作指令
