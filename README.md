# 概要
ロボマスを16台同時に制御できるパッケージである。
このROS2パッケージは「robomas_interfaces」パッケージと一緒に使うことを想定している。

# 電流値について
## C620の場合
- 最大定常電流は20A

## C610の場合
- 最大定常電流は10A

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
- target → 電流制御の場合：ロボマスに流す電流[mA]、　速度制御：回転子の角速度[rpm]、　位置制御：回転子の累積回転量[° (度)]

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

# PIDパラメータの調整について
カスケード制御（位置制御ループの中に速度制御ループが入っている構造）を採用している。PIDパラメータを調整する際は速度制御のPIDパラメータを決めてから位置制御のPIDパラメータを調整する。
マイコン内ではこのように計算がされている。
$$
\underbrace{u(t)}_{\text{出力}} = \underbrace{K_p \cdot e(t)}_{\text{P項}} + \underbrace{K_i \sum e(t) \Delta t}_{\text{I項}} + \underbrace{K_d \frac{e(t) - e(t-\Delta t)}{\Delta t}}_{\text{D項}}
$$

- $e(t)$: 目標値との偏差（Target - Current）
- dt: 制御周期（本システムでは 2ms = 0.002s）

PIDパラメータを変更するには`/config/robomas_params.yaml`の中身を変更する。

## パラメータの意味と調整のコツ（経験則）
### 速度制御
- speed_kp : あげすぎるとガガガと不穏な音が鳴る。下げすぎると応答性が良くない。大体5.0 ~ 20.0くらい
- speed_ki : 負荷がかかった時に速度が落ちるのを防ぐ。大きければ大きい方がいいと先輩が言ってた。この大きさは正直わからん。どれくらいでもいい感じがする。
- speed_kd : 急激な速度変化による振動を抑えるらしい。あってもいいが小さくするべき。なくてもいいまである。大きさは0.01以下かな
- speed_limit : 全ての計算後の出力の上限となる値。流していい最大の電流値をここで決める。最大定格電流は「電流値について」の章を参照。M3508は特殊で、16384で20,000mAを表す。M2006はそのまま。最大定格以上の値を入れないように。

### 位置制御
- pos_kp : 大体10以下
- pos_ki : あと一度だけ届かない〜　とかで使われるんじゃないかな？ 1くらいかそれ以下かな
- pos_kd : ピタッと止まるために必要。大体0.1とか？
- pos_limit : **速度**の最大値。めちゃ大事。これで位置制御のロボマスが目標まで移動している時の速度の最大値を決める。これが適当に大きな値とかになっていると、位置制御で目標値を変えた瞬間にとんでもなく爆速で動き始めて非常に危険。

### 例
```bash
speed_kp: 8.0
speed_ki: 40.0
speed_kd: 0.01
speed_i_limit: 1000.0
speed_limit: 10000.0
pos_kp: 8.0
pos_ki: 1.0
pos_kd: 0.01
pos_i_limit: 100
pos_limit: 1000.0
```

# 備考
M3508はユーザーからもらった電流値[mA]を生値（例えば20Aなら16384）に変換している。フィードバックの電流値も同様である。
