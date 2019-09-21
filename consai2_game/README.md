# consai2_game

ロボット・ボールの位置情報をもとにゲーム戦略を計算するパッケージです。

# Example

[`scripts/example/game.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/game.py)
がサンプルコードです。

## 概要

ロボット・ボールの位置情報とレフェリー信号をもとに、
ロボットの行動を決定し、ロボットの移動目標位置やキック信号を生成します。

## 役割の割り当て

自チームのロボットにはキーパー・アタッカー・ディフェンスの役割を与えます。

[`role.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/role.py)
が役割分担の機能を持ちます。

ロボットの交代や、ボールとロボットの位置関係に合わせて役割を切り替えます。

## フィールド幾何学情報の変換

[`field.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/field.py)
は、SSL-Visionから受け取ったフィールド幾何学情報をconsai2_gameで使用できるデータ形式に変換します。

## フィールド状況の解析

[`observer.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/observer.py)
はフィールド状況を観測・解析します。

ボールの移動判定もここで行われます。

## レフェリー信号の解析

レフェリーはチームカラー(blue, yellow)宛にキックオフやインダイレクト等の信号を送ります。
この信号を自チーム・相手チームの信号に解釈するのが[`referee_wrapper.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/referee_wrapper.py)
です。

## 衝突回避位置の生成

試合中はロボット同士の衝突を避けなければなりません。
その衝突回避位置を生成するのが[`avoidance.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/avoidance.py)です。

ここでは、ロボットに最も近い障害物の近くに移動経由点を生成する、
シンプルな衝突回避アルゴリズムを計算しています。

## ゲーム戦略の構造

`game.py`ではロボットごとに`RobotNode`のインスタンスを与え、
毎ループ(60fps)各`RobotNode`を更新します。

`RobotNode`はレフェリー信号と、自身が持つ役割をもとに行動`action`を生成します。

## actions/goalie

[`actions/goalie.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/actions/goalie.py)
はゴールキーパの`action`を生成します。

ボール位置や相手ロボットの位置に合わせて、ゴールキーパの行動を決定します。

## actions/defense

[`actions/defense.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/actions/defense.py)
はデフェンスロボットの`action`を生成します。

デフェンスにはディフェンスエリア前を防御するものと、ゾーンディフェンスの2種類の`action`があります。

## actions/offense

[`actions/offense.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/actions/offense.py)
はオフェンスロボットの`action`を生成します。

セットプレイやインプレイでボールを蹴る行動を決定します。

## actions/normal

[`actions/normal.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/actions/normal.py)
は試合には直接影響しない行動を生成します。

指定座標への移動や、ロボットの停止等の行動を決定します。

## actions/ball_placement

[`actions/ball_placement.py`](https://github.com/SSL-Roots/consai2/blob/master/consai2_game/scripts/example/actions/ball_placement.py)
はボールプレースメントの行動を生成します。

セットプレイ開始時は、ロボットがボールをセットプレイ開始場所へ運ばなくてはなりません。
`ball_placement.py`がその行動を決定します。

## Subscribe topics

- `vision_receiver/raw_vision_geometry` (consai2_msgs/VisionGeometry)
- `referee_wrapper/decoded_referee` (consai2_msgs/DecodedReferee)
- `vision_wrapper/ball_info` (consai2_msgs/BallInfo)
- `vision_wrapper/robot_info_*` (consai2_msgs/RobotInfo)

## Publish topics

- `~/control_target_*` (consai2_msgs/ControlTarget)
  - ロボットの行動目標値
  - 移動経路やキック・ドリブルのフラグ等を含む。
