# consai2_receiver

Vision、Referee、Robotからの情報を受けとるパッケージです。


## ノードの起動方法

次のコマンドでvision_receiverとreferee_receiverが起動します。

```sh
roslaunch consai2_receiver receiver.launch 
```

## マルチキャストアドレス・ポートの変更方法

`consai2_description/param/game.yaml`を編集します。

## チームサイドの変更方法

`consai2_description/param/game.yaml`を編集します。

## Publish Topics

- vision_receiver
  - raw_vision_detections (consai2_msgs/VisionDetections)
    - 各カメラで撮影されたロボットとボールの座標・角度
  - raw_vision_geometry (consai2_msgs/VisionGeometry)
    - Visionで設定されたフィールド形状
- referee_receiver
  - raw_referee (consai2_msgs/Referee)
    - Refereeが出力する信号


## 参考ページ
### Goole proto files

- Vision
  - https://github.com/RoboCup-SSL/ssl-vision/tree/master/src/shared/proto
- Referee
  - https://github.com/RoboCup-SSL/ssl-refbox
