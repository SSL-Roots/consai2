# consai2_examples

consai2のサンプルコード集です。

## 事前準備

### サンプル実行環境の準備

シミュレータでサンプルを実行する場合は[grSim](https://github.com/RoboCup-SSL/grSim)を起動してください。

実機ロボットを操作する場合は[consai2_sender](https://github.com/SSL-Roots/consai2/tree/master/consai2_sender)
の[real_sender.py](https://github.com/SSL-Roots/consai2/blob/master/consai2_sender/scripts/example/real_sender.py)
を参考に、データ送信ノードを事前に作成してください。

Referee信号を扱うサンプルでは、[SSL-Game-Controller](https://github.com/RoboCup-SSL/ssl-game-controller)を起動してください。

### パラメータの設定

Vision, Referee, シミュレータとの通信で扱うマルチキャストアドレス・ポートを設定してください。

パラメータは[consai2_description](https://github.com/SSL-Roots/consai2/tree/master/consai2_description)
の[game.yaml](https://github.com/SSL-Roots/consai2/blob/master/consai2_description/param/game.yaml)
を編集することで設定できます。

---

## Visualizer 

ビジュアライザでフィールド・ロボット・ボールを描画するサンプルです。
Refereeの信号も受信します。

VisionとRefereeからデータを受信できるか確認したいときに実行すると良いです。

次のコマンドでノードを起動します。

```sh
roslaunch consai2_examples visualizer.launch
```

![consai2_visualizer](https://github.com/SSL-Roots/consai2/blob/images/images/consai2_visualizer.png "consai2_visualizer")

---

## Joystick

ジョイスティックでロボットを操縦するコード例です。

次のコマンドでノードを起動します。

```sh
roslaunch consai2_examples joystic_example.launch
```

### 実機とgrSimの切り替え方法

`consai2_examples/launch/joystic_example.launch`を編集するか、
launchファイルの引数で設定できます。

```sh
# 例:実機を動かす場合
roslaunch consai2_examples joystic_example.launch sim:=false

# 例:grSimのロボットを動かす場合
roslaunch consai2_examples joystic_example.launch sim:=true
```

### 制御の切り替え方法


```sh
# 例:consai2_controlを経由する場合
roslaunch consai2_examples joystic_example.launch direct:=false

# 例:consai2_controlを経由せず、直接操縦する場合
roslaunch consai2_examples joystic_example.launch direct:=true
```

### キー割り当ての変更

デフォルトのキー割り当てはこちらです。
ジョイスティックは[Logicool Wireless Gamepad F710](https://support.logicool.co.jp/ja_jp/product/wireless-gamepad-f710)
を使っています。  

![Key_Config](https://github.com/SSL-Roots/consai2/blob/images/images/key_config_direct.png)

`consai2_examples/launch/joystic_example.launch`のキー番号を編集することで、キー割り当てを変更できます。  

デフォルトのキー番号はこちらです。

![Key_Numbers](https://github.com/SSL-Roots/consai2/blob/images/images/key_numbders.png)

### 使用するジョイスティックの変更

[PS4のコントローラー(DUALSHOCK 4)](https://www.jp.playstation.com/accessories/dualshock4/)を使用することもできます。  
`consai2_examples/launch/joystic_example.launch`のloadするファイルを変更してください。

```
<!-- Logicool Wireless Gamepad F710を使用する場合 -->
<rosparam command="load" file="$(find consai2_examples)/param/joy_f710.yaml" />

<!-- DUALSHOCK 4を使用する場合 -->
<rosparam command="load" file="$(find consai2_examples)/param/joy_ps4.yaml" />
```

---

## Control

consai2_controlを使うコード例です。

次のコマンドでノードを起動します。

```sh
# 実機を動かす場合
roslaunch consai2_examples control_example.launch sim:=false

# grSimのロボットを動かす場合
roslaunch consai2_examples control_example.launch sim:=true
```

その他、パラメータを変更することで制御対象・制御方式を変更できます。

```sh
# 制御対象のロボットID、チームカラーの変更
roslaunch consai2_examples control_example.launch id:=2 color:=yellow

# 速度制御のサンプル
roslaunch consai2_examples control_example.launch velocity_control:=true

```

複数のロボットを動かす場合は、次のコマンドを実行します。

このサンプルではRefereeのSTOP信号でロボットが動作します。
Refereeのアプリケーションを立ち上げてください。

```sh
# 例：grSimのロボットを動かす場合
roslaunch consai2_examples control_example.launch sim:=true collective:=true
```

![consai2_control_example](https://github.com/SSL-Roots/consai2/blob/images/images/consai2_control_example.png "consai2_control_example")

---

## Game

試合用のサンプルです。

次のコマンドでノードを起動します。

```sh
# 実機を動かす場合
roslaunch consai2_examples game.launch sim:=false

# grSimのロボットを動かす場合
roslaunch consai2_examples game.launch sim:=true
```

### CON-SAI2 vs CON-SAI2

次のコマンドで、CON-SAI2同士を戦わせることができます。

Visionのマルチキャストアドレスに
ローカルループバックアドレスを設定していると正常動作しない場合があるので注意してください。

```sh
# 青チームで左守り
roslaunch consai2_examples game.launch sim:=true color:=blue side:=left

# 別の端末で
# 黄チームで右守り
roslaunch consai2_examples game.launch sim:=true color:=yellow side:=right
```

