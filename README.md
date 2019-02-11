[![Build Status](https://travis-ci.org/SSL-Roots/consai2.svg?branch=master)](https://travis-ci.org/SSL-Roots/consai2)

# CON-SAI2

CON-SAIの修正版


## CON-SAIとの違い

- ROSのルールに合うようパッケージ名を修正しました


## Installation


ああああ


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
