# consai2_receiver

Vision、Referee、Robotからの情報を受けとるパッケージです。


## ノードの起動方法

次のコマンドでvision_receiverとreferee_receiverが起動します。

```sh
roslaunch consai2_receiver receiver.launch 
```

## マルチキャストアドレス・ポートの変更方法

`consai2_receiver/launch/receiver.launch`を編集するか、
launchファイルの引数で設定できます。

```sh
# 例:Visionのアドレス・ポートを変更
roslaunch consai2_receiver receiver.launch vision_addr:=224.5.23.2 vision_port:=10006

# 例:Refereeのアドレス・ポートを変更
roslaunch consai2_receiver receiver.launch referee_addr:=224.5.23.2 referee_port:=10006
```

## チームサイドの変更方法

`consai2_receiver/launch/receiver.launch`を編集するか、
launchファイルの引数で設定できます。

```sh
# 例：左守り、右攻め
roslaunch consai2_receiver receiver.launch side:=left

# 例：右守り、左攻め
roslaunch consai2_receiver receiver.launch side:=right
```

## Parameters
- vision_receiver
  - ~/vision_addr
    - Visionのマルチキャストアドレス
    - default: 224.5.23.2
  - ~/vision_port
    - Visionのポート番号
    - default: 10006
  - ~/side
    - 自チームサイド
    - default: left
- referee_receiver
  - ~/referee_addr
    - Refereeのマルチキャストアドレス
    - default: 224.5.23.1
  - ~/referee_port
    - Refereeのポート番号
    - default: 10003

## Publish Topics

- vision_receiver
  - raw_vision_detections
    - 各カメラで撮影されたロボットとボールの座標・角度
  - raw_vision_geometry
    - Visionで設定されたフィールド形状
- referee_receiver
  - raw_referee
    - Refereeが出力する信号


## 参考ページ
### Goole proto files

- Vision
  - https://github.com/RoboCup-SSL/ssl-vision/tree/master/src/shared/proto
- Referee
  - https://github.com/RoboCup-SSL/ssl-refbox
