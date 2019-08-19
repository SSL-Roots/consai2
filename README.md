[![Build Status](https://travis-ci.org/SSL-Roots/consai2.svg?branch=master)](https://travis-ci.org/SSL-Roots/consai2)

# CON-SAI2

CON-SAI2は[RoboCup SSL](https://ssl.robocup.org/)に
初めて参加する人でも開発できるサッカーAIです。

**CON**tribution to **S**occer **AI**

![consai2_main](https://github.com/SSL-Roots/consai2/blob/images/images/consai2_main.png "consai2_main")

前作の[CON-SAI](https://github.com/SSL-Roots/CON-SAI)
からCON-SAI2へ開発を移行しました。

### CON-SAIとの違い

- ROSのルールに合うようパッケージ名を修正しました
- サンプルプログラム([consai2_examples](https://github.com/SSL-Roots/consai2/tree/master/consai2_examples))を追加しました
  - Visualizer（Vision, Refereeとの通信確認）
  - Control（走行制御のテスト）
  - Game（試合プログラム）
  - Joystick（ジョイスティックコントローラによるロボットの操縦）
- ユーザが拡張すべきプログラムと、変更が不要なプログラムを明確にしました
  - 例えば、Vision・Refereeのデータ受信やシミュレータへのデータ送信プログラムは変更不要です
  - 例えば、戦略プログラムや走行制御、ロボットへのデータ送信プログラムはユーザに合わせて拡張してください
  - Rootsが作成したコード例を各パッケージのexampleディレクトリに用意しています

## Requirements
CON-SAI2はUbuntu 18.04で作成・テストしてます。

下記のPCスペックで問題なく動作します。
- Intel(R) Core(TM) i5-6600K CPU @ 3.50GHz
- 8 GB of RAM
- 有線LANポート（試合会場では有線LANでロボット・ボール位置座標データを受信します)

## Installation

### ROSのインストール
[ROS (Robot Operating System)](http://wiki.ros.org/ja)
は、ロボットソフトウェア開発をサポートする
ライブラリ・ツールが豊富に含まれたオープンソースソフトウェアです。

CON-SAI2はKinetic、Melodicに対応しています。

[**Ubuntu install of ROS Melodic**](
http://wiki.ros.org/melodic/Installation/Ubuntu
)を参考に、インストールしてください。フルインストール推奨です。


### Google Protobuf Librariesのインストール

```zsh
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler
sudo apt-get install python-pip
sudo pip2 install protobuf
```

### 依存パッケージのインストール
```zsh
sudo apt-get install ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-bfl -y
```

### CON-SAI2のビルド

```zsh
git clone https://github.com/SSL-Roots/consai2 ~/catkin_ws/src/consai2
cd ~/catkin_ws/src/
rosdep install -r -y --from-paths . --ignore-src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## RoboCup SSLのAI開発に必要なツールをインストール

RoboCup SSLのAI開発にはシミュレータ(grSim)と審判ソフト(SSL-Game-Controller)があると便利です。

下記ページを参考に、ソフトウェアをインストールしてください。

[grSim](https://github.com/RoboCup-SSL/grSim)

[SSL-Game-Controller](https://github.com/RoboCup-SSL/ssl-game-controller)

## Tutorial

CON-SAI2を始めるに当たって、まずはサンプルプログラムを実行してください。

[consai2_examples](https://github.com/SSL-Roots/consai2/tree/master/consai2_examples)
にサンプロプログラムの実行方法が記載されています。

## Development

いくつかのパッケージ内の`example`ディレクトリはRootsのコード例です。そのため、今後もRootsの意思で中身が大幅に変わる可能性があります。

`master`ブランチの変更を容易にマージするため、以下項目を守ってください。

- `master`からブランチを切り分ける
- 各パッケージの`example`ディレクトリ内を編集しない
- 各パッケージの`src`、`scripts`ディレクトリ直下にユーザコードを追加する

## Contribution

CON-SAI2は全ユーザ共通で使用できるコード(`consai2_receiver`や`consai2_sender`など)と、
各ユーザで作成すべきコード(作成例：`example`ディレクトリ)に分かれています。

全ユーザ共通で扱うコードについてはRoots以外の要望も受け入れたいので、
**Contribution(issue, pullreq)をお願いします。**

些細な修正や指摘でも良いので、まずはissueの作成から始めてみてください。

## Author

CON-SAI2はRoboCup SSLに参加している日本人チーム**Roots**が作成しています。

RoboCup SSLへの参加方法、ロボットに必要な機能、開発環境などは
Rootsのホームページに記載してます。

[Roots - Home](https://github.com/SSL-Roots/Roots_home/wiki)


