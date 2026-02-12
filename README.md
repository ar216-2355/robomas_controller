# 概要
ロボマスを16台同時に制御できるパッケージである。
このROS2パッケージは「robomas_interfaces」パッケージと一緒に使うことを想定している。

# 電流値について
## C620の場合
- 最大定常電流は20A
- -16384.0 ~ 16384.0 の間の数値を入力することができる
- 16384 が 20A である

## C610の場合
- 最大定常電流は10A
- -10000.0 ~ 10000.0 の間の数値を入力することができる
- 10000.0 が 10A である

# PID設定値を変更した場合
1. ファイルを保存
1. ビルド

# 各種コマンド
### 実行
```bash
ros2 launch robomas_controller robomas_bridge.launch.py
```

### ロボマスを動かすコマンド
- motor_id → ロボマスの番号
- mode → 0：電流制御、　1：速度制御、　2：位置制御
- target → 電流制御の場合：２章「電流値について」を参照、　速度制御：回転子の角速度[rpm]、　位置制御：回転子の累積回転量[° (度)]

ロボマス５を速度制御で300rpmで回すコマンド
```bash
ros2 topic pub --once /robomas/cmd robomas_interfaces/msg/RobomasPacket "{motors: [{motor_id: 5, mode: 1, target: 300.0}]}"
```

複数のロボマスのモードと目標値を一気に設定するコマンド
```bash
ros2 topic pub --once /robomas/cmd robomas_interfaces/msg/RobomasPacket "{
  motors: [
    {motor_id: 1, mode: 0, target: 300.0},
    {motor_id: 2, mode: 2, target: -500.0},
    {motor_id: 3, mode: 1, target: 30000.0}
  ]
}"
```