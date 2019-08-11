# consai2_examples

consai2のサンプルコード集です

## Visualizer 

ビジュアライザでフィールド・ロボット・ボールを描画するサンプルです。
Refereeの信号も受信します。

VisionとRefereeからデータを受信できるか確認したいときに実行すると良いです。

次のコマンドでノードを起動します。

```sh
roslaunch consai2_examples visualizer.launch
```

## control_example

consai2_controlを使うコード例です。

次のコマンドでノードを起動します。

```sh
# 実機を動かす場合
roslaunch consai2_examples control_example.launch sim:=false

# grSimのロボットを動かす場合
roslaunch consai2_examples control_example.launch sim:=true
```

複数のロボットを動かす場合は、次のコマンドを実行します。

このサンプルではRefereeのSTOP信号でロボットが動作します。
Refereeのアプリケーションを立ち上げてください。

```sh
# 例：grSimのロボットを動かす場合
roslaunch consai2_examples control_example.launch sim:=true collective:=true
```

## Game

試合用のサンプルです。

事前に`consai2_description/param/game.yaml`を編集し、必要なパラメータを設定してください。

次のコマンドでノードを起動します。

```sh
# 実機を動かす場合
roslaunch consai2_examples game.launch sim:=false

# grSimのロボットを動かす場合
roslaunch consai2_examples game.launch sim:=true
```

## joystic_example

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
roslaunch consai2_examples joystic_example.launch direct:=tru
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