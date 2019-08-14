[![Build Status](https://travis-ci.org/SSL-Roots/consai2.svg?branch=master)](https://travis-ci.org/SSL-Roots/consai2)

# CON-SAI2

CON-SAIの修正版


## CON-SAIとの違い

- ROSのルールに合うようパッケージ名を修正しました


## Installation

### ROSのインストール

Kinetic以上に対応しています。

http://wiki.ros.org/kinetic/Installation/Ubuntu

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

### consai2のビルド

```zsh
git clone https://github.com/SSL-Roots/consai2 ~/catkin_ws/src/consai2
cd ~/catkin_ws/src/
rosdep install -r -y --from-paths . --ignore-src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## テスト

```zsh
# Run the all tests
cd ~/catkin_ws
catkin_make run_tests 
  
# Get result
# Caution! `catkin_make run_tests` always returns 0
catkin_test_results
  
# Run arbitary test
cd ~/catkin_ws
catkin_make run_tests_consai2_receiver_nosetests_scripts.tests.test_format_convert.py

```
