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

`consai2_description/param/game.yaml`を編集します。

## 実機送信機のデバイス・ボーレートの変更方法

`consai2_description/param/game.yaml`を編集します。

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
