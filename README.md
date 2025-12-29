# minicar_diff_hardware

ROS2用のミニカー実機ハードウェアインターフェースパッケージです。PCA9685を使用したモーター制御とRPLiDARセンサーをサポートします。

## 概要

このパッケージは以下の機能を提供します：

- ros2_control対応ハードウェアインターフェース
- PCA9685 PWMコントローラーによるモーター制御
- GPIOベースのモーター制御（バックアップ）
- RPLiDAR C1センサーサポート
- 差動駆動ロボット制御

## アーキテクチャ

### 主要コンポーネント

- **MinicarHardwareInterface**: ros2_controlシステムインターフェース
- **PCA9685Controller**: I2C経由PCA9685制御クラス
- **GPIOMotorController**: 直接GPIO制御クラス（フォールバック用）

### ハードウェア構成

- **モーターコントローラー**: PCA9685 (I2C: 0x40)
- **LiDAR**: RPLiDAR C1
- **プラットフォーム**: Raspberry Pi 4

## 使用方法

### 基本的な起動

```bash
# 実機ロボット完全起動（RPLiDAR含む）
ros2 launch minicar_diff_hardware real_robot_bringup.launch.py

# モーター制御のみ
ros2 launch minicar_diff_hardware motor_only.launch.py
```

### launchファイル

#### `real_robot_bringup.launch.py`

実機ロボットの完全なシステムを起動します。

**使用可能なオプション：**

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `robot_ns` | `real_robot` | ロボットの名前空間 |
| `use_sim_time` | `false` | シミュレーション時間の使用 |
| `rplidar_serial_port` | `/dev/ttyUSB0` | RPLiDARのシリアルポート |

**含まれるノード：**
- Robot State Publisher
- ros2_control Hardware Interface
- Joint State Broadcaster
- Diff Drive Controller
- RPLiDAR Node
- Static TF (base_link → laser)

**使用例：**

```bash
# デフォルト設定で起動
ros2 launch minicar_diff_hardware real_robot_bringup.launch.py

# 異なるシリアルポートを指定
ros2 launch minicar_diff_hardware real_robot_bringup.launch.py rplidar_serial_port:=/dev/ttyUSB1

# 名前空間を変更
ros2 launch minicar_diff_hardware real_robot_bringup.launch.py robot_ns:=my_robot
```

#### `motor_only.launch.py`

モーター制御システムのみを起動します（センサーなし）。

**使用可能なオプション：**

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `robot_ns` | `real_robot` | ロボットの名前空間 |
| `use_sim_time` | `false` | シミュレーション時間の使用 |

**含まれるノード：**
- Robot State Publisher
- ros2_control Hardware Interface
- Joint State Broadcaster  
- Diff Drive Controller

## ハードウェア設定

### PCA9685接続

```
PCA9685チャンネル配置：
- チャンネル 0, 1: 左前モーター（PWM、方向）
- チャンネル 2, 3: 左後モーター（PWM、方向）
- チャンネル 4, 5: 右前モーター（PWM、方向）
- チャンネル 6, 7: 右後モーター（PWM、方向）
```

### I2C設定

Raspberry Piでi2cを有効化：

```bash
# i2c-toolsインストール
sudo apt install i2c-tools

# I2Cデバイス確認
i2cdetect -y 1

# 期待される出力: アドレス0x40でPCA9685が検出される
```

### コントローラーパラメータ

`config/minicar_hardware_config.yaml`で設定：

```yaml
diff_drive_controller:
  ros__parameters:
    wheel_separation: 0.195  # ホイール間距離 (m)
    wheel_radius: 0.035      # ホイール半径 (m)
    cmd_vel_timeout: 0.5     # 速度コマンドタイムアウト (s)
    publish_rate: 50.0       # オドメトリパブリッシュレート (Hz)
```

## ロボット制御

起動後、以下のトピックでロボットを制御できます：

```bash
# 速度指令の送信
ros2 topic pub /{robot_ns}/diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'

# オドメトリ情報の確認
ros2 topic echo /{robot_ns}/odom

# ジョイント状態の確認
ros2 topic echo /{robot_ns}/joint_states

# LiDARデータの確認
ros2 topic echo /{robot_ns}/scan
```

## 依存関係

### システム要件
- Ubuntu 22.04 + ROS2 Humble
- Raspberry Pi 4 (推奨)
- PCA9685 PWMコントローラー
- RPLiDAR C1

### ROS2パッケージ
- hardware_interface
- controller_manager
- diff_drive_controller
- joint_state_broadcaster
- robot_state_publisher
- rplidar_ros2

### システムパッケージ
```bash
sudo apt install libi2c-dev i2c-tools
```

## トラブルシューティング

### I2C通信エラー

```bash
# I2Cデバイスの権限確認
ls -l /dev/i2c-1
sudo chmod 666 /dev/i2c-1

# ユーザーをi2cグループに追加
sudo usermod -a -G i2c $USER
```

### RPLiDARが検出されない

```bash
# シリアルポート確認
ls -l /dev/ttyUSB*

# 権限設定
sudo chmod 666 /dev/ttyUSB0
```

### モーターが動作しない

1. PCA9685の電源とI2C接続を確認
2. `i2cdetect -y 1`でアドレス0x40を確認
3. ログでハードウェアインターフェースの初期化状況を確認

### コントローラーが起動しない

```bash
# Controller managerの状態確認
ros2 control list_controllers

# コントローラーの手動起動
ros2 run controller_manager spawner diff_drive_controller --controller-manager /{robot_ns}/controller_manager
```

## 開発者向け

### ビルド方法

```bash
cd ~/ros2_ws
colcon build --packages-select minicar_diff_hardware
source install/setup.bash
```

### ハードウェアインターフェースのカスタマイズ

- `src/minicar_hardware_interface.cpp`: メインインターフェース
- `src/pca9685_controller.cpp`: PCA9685制御ロジック  
- `src/gpio_motor_controller.cpp`: GPIO制御ロジック