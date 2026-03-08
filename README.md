# LaMa ROS 2 - IRIS LaMa の ROS 2 Humble 対応版

## 概要
LaMa (Localization and Mapping) は、アヴェイロ大学の Intelligent Robotics and Systems (IRIS) 研究所によって開発された 2D SLAM および自己位置推定パッケージです。非常に軽量で、Raspberry Pi 3 などのリソースの限られた環境でも高速に動作します。

本リポジトリは、オリジナルの ROS 2 版を **ROS 2 Humble** および **Ubuntu 22.04** に対応させ、さらに最新の ROS 2 ベストプラクティス（コンポーネント化など）を取り入れたアップデート版です。

## 主な変更点 (Eloquent -> Humble)
- **コンポーネント対応**: 各ノードを `rclcpp::Node` 継承クラスとして再設計し、ROS 2 コンポーネントコンテナへのロードに対応しました。
- **Humble API 準拠**: TF2、MessageFilter、QoS 設定などを Humble の最新 API に合わせて修正しました。
- **実行形式の柔軟性**: `ros2 run` によるスタンドアロン起動とコンポーネントとしてのロードの両方に対応しました。
- **C++17 への移行**: ROS 2 Humble の標準に合わせ、C++17 でビルドされます。
- **Launch ファイルの最適化**: インストール環境に依存せず設定ファイルを正しく参照するよう、モダンな Launch 記述に改善しました。

## インストール方法
このパッケージを利用するには、コアライブラリである `iris_lama` が必要です。

```bash
# ワークスペースの作成
mkdir -p ~/lama_ws/src
cd ~/lama_ws/src

# 依存ライブラリと本パッケージのクローン
git clone https://github.com/iris-ua/iris_lama.git
git clone https://github.com/iris-ua/iris_lama_ros.git  # 本リポジトリ
cd ..

# ビルド
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 提供ノード

### 1. Online SLAM (`slam2d_ros`)
逐次的にマップを生成する標準的な SLAM です。
```bash
ros2 launch iris_lama_ros2 slam2d_live_launch.py
```

### 2. Particle Filter SLAM (`pf_slam2d_ros`)
パーティクルフィルタを用いた SLAM です。
```bash
ros2 launch iris_lama_ros2 pf_slam2d_live_launch.py
```

### 3. 2D Localization (`loc2d_ros`)
既存のマップ（`/map` サービス経由）を利用した自己位置推定です。
```bash
ros2 launch iris_lama_ros2 loc2d_launch.py
```
※起動後、RViz2 の "2D Pose Estimate" を使用して初期位置を与えてください。現時点ではグローバル自己位置推定（自動初期化）は未実装です。

## 主なパラメータ
共通の主要パラメータ：
- `global_frame_id`: マップフレーム (default: "map")
- `odom_frame_id`: オドメトリフレーム (default: "odom")
- `base_frame_id`: ロボットのベースフレーム (default: "base_link")
- `scan_topic`: レーザースキャンデータのトピック (default: "/scan")
- `resolution`: マップの解像度 [m] (default: 0.05)
- `d_thresh`: 更新を行う移動距離の閾値 [m] (SLAM: 0.01, PF: 0.5)
- `a_thresh`: 更新を行う回転角度の閾値 [rad] (SLAM: 0.25, PF: 0.25)
- `map_publish_period`: マップを配信する周期 [秒] (default: 5.0)

## オフラインマッピング (rosbag)
`ros2 bag` を再生しながら高速に地図を生成することも可能です。
`launch/slam2d_offline_launch.py` 等の `bag_file` パスを編集して実行してください。
```bash
ros2 launch iris_lama_ros2 slam2d_offline_launch.py
```

---
(C) 2019 Eurico Pedrosa, University of Aveiro.
Ported to ROS 2 by David Simoes, University of Aveiro.
Updated for ROS 2 Humble by Contributors.
