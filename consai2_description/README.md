# consai2_description

CON-SAI2のパッケージ間で共有するROSパラメータを設定します。

## パラメータの設定方法

次のコマンドでlaunchファイルを起動します。

```sh
roslaunch consai2_description description.launch
```

## パラメータ編集方法

`consai2_description/param`内のyamlファイルを編集します。

## Parameters
- game.yaml
  - consai2_description/max_id (integer, default:15)
    - ゲームに使用できる最大ID
  - consai2_description/player_num (integer, default:8)
    - フィールドに同時に出せる自チームのロボット数
  - consai2_description/our_side (string, default:'left')
    - フィールドの自チームディフェンス側 ('left' or 'right')
  - consai2_description/our_color (string, default:'blue')
    - 自チームのマーカ色 ('blue' or 'yellow')
- geometry.yaml
  - robot_radius (integer, default:0.09)
    - ロボットの半径 (unit:meter)
  - ball_radius (integer, default:0.0215)
    - ボールの半径 (unit:meter)

