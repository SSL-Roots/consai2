# consai2_sender

ロボット(実機 / grSim)へ動作司令を送信するパッケージです。

## ノードの起動方法

次のコマンドでsenderが起動します。

```sh
roslaunch consai2_sender sender.launch 
```

## 実機 / grSim の切り替え方法

`consai2_sender/launch/sender.launch`を編集するか、
launchファイルの引数で設定できます。

```sh
# 例:実機のロボットに動作司令を送信する
roslaunch consai2_sender sender.launch sim:=false

# 例:grSimのロボットに動作司令を送信する
roslaunch consai2_sender sender.launch sim:=true
```

## grSimのサーバアドレス・ポートの変更方法

`consai2_sender/launch/sender.launch`を編集するか、
launchファイルの引数で設定できます。

```sh
# 例:grSimのアドレス・ポートを変更
roslaunch consai2_sender sender.launch sim:=true server_addr:=127.0.0.1 server_port:=20011
```

## 実機送信機のデバイス・ボーレートの変更方法

`consai2_sender/launch/sender.launch`を編集するか、
launchファイルの引数で設定できます。

```sh
# 例:実機送信機のデバイス・ボーレートの変更方法
roslaunch consai2_sender sender.launch sim:=false device:=/dev/ttyUSB0 baudrate:=57600
```

## Parameters
- sim_sender
  - ~/server_addr (string, default:'127.0.0.1')
    - grSimのサーバアドレス
  - ~/server_port (integer, default:20011)
    - grSimのポート番号
- real_sender 
  - ~/device (string, default:'/dev/ttyUSB0')
    - 実機送信機のデバイス
  - ~/baudrate (integer, default:57600)
    - 実機送信機のボーレート

## Subsribe Topics
- sim_sender
  - consai2_control/robot_commands' (consai2_msgs/RobotCommands)
    - ロボットの動作司令
- real_sender
  - consai2_control/robot_commands' (consai2_msgs/RobotCommands)
    - ロボットの動作司令

## 参考ページ
### Google proto files

- grSim
  - https://github.com/RoboCup-SSL/grSim/tree/master/src/proto
