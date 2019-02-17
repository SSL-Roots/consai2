# consai2_sender

ロボット(Real / Sim)へ動作司令を送信するパッケージです。

## ノードの起動方法

次のコマンドでsenderが起動します。

```sh
roslaunch consai2_sender sender.launch 
```

## Real / Sim の切り替え方法

`consai2_sender/launch/sender.launch`を編集するか、
launchファイルの引数で設定できます。

```sh
# 例:実世界のロボットに動作司令を送信する
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

## Parameters
- sim_sender
  - ~/server_addr
    - grSimのサーバアドレス
    - default: 127.0.0.1
  - ~/server_port
    - grSimのポート番号
    - default: 20011

## Subsribe Topics
- sim_sender
  - consai2_control/robot_commands'
    - ロボットの動作司令

## 参考ページ
### Google proto files

- grSim
  - https://github.com/RoboCup-SSL/grSim/tree/master/src/proto
