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

## 各ノードの詳細と使い方

### 1. Online SLAM (`slam2d_ros`)
逐次的にマップを生成する標準的な SLAM です。

**起動方法:**
```bash
ros2 launch iris_lama_ros2 slam2d_live_launch.py
```

**主なパラメータ:**
- `global_frame_id` (string, default: "map"): マップフレームのID。
- `odom_frame_id` (string, default: "odom"): オドメトリフレームのID。
- `base_frame_id` (string, default: "base_link"): ロボット本体のフレームID。
- `scan_topic` (string, default: "/scan"): 購読するレーザースキャンのトピック名。
- `initial_pos_x/y/a` (double, default: 0.0): 起動時の初期位置（x, y）と向き（a: 角度[rad]）。
- `d_thresh` (double, default: 0.01): 地図更新を行う最小移動距離 [m]。
- `a_thresh` (double, default: 0.25): 地図更新を行う最小回転角度 [rad]。
- `resolution` (double, default: 0.05): マップの解像度 [m/cell]。
- `map_publish_period` (double, default: 5.0): マップ（OccupancyGrid）を配信する周期 [秒]。
- `mrange` (double, default: 16.0): レーザースキャンの最大有効距離 [m]。
- `use_compression` (bool, default: false): マップデータの圧縮を行うかどうか。

---

### 2. Particle Filter SLAM (`pf_slam2d_ros`)
パーティクルフィルタを用いた、より堅牢な SLAM です。

**起動方法:**
```bash
ros2 launch iris_lama_ros2 pf_slam2d_live_launch.py
```

**SLAM固有のパラメータ:**
- `particles` (int, default: 30): 使用するパーティクル数。
- `threads` (int, default: -1): 使用するスレッド数（-1: 無効, 0: 自動）。
- `d_thresh` (double, default: 0.5): PF版の移動距離閾値（標準版より大きめを推奨）。
- `srr / str / stt / srt` (double): オドメトリの誤差モデルパラメータ。
- `sigma` (double, default: 0.05): 計測の標準偏差。
- `lgain` (double, default: 3.0): 尤度計算のゲイン。

---

### 3. 2D Localization (`loc2d_ros`)
既存の地図を使用して自己位置推定のみを行います。`/map` サービスを提供するノード（`nav2_map_server`など）が必要です。

**起動方法:**
```bash
ros2 launch iris_lama_ros2 loc2d_launch.py
```

**使い方と初期位置設定:**
1. ノードを起動すると、パラメータ `initial_pos_x/y/a` で指定された位置（デフォルトは `0,0,0`）で推定を開始します。
2. 実際のロボットの位置がズレている場合は、**RViz2 の "2D Pose Estimate" ボタン** を使用して地図上の正しい位置と向きを指定してください。
3. 指定された瞬間、内部の推定ポーズがリセットされ、新しい位置から追従を開始します。

---

## パラメータ設定のヒント
- **初期位置の固定**: ロボットの開始位置が常に決まっている場合は、YAMLファイルで `initial_pos_x/y/a` を設定しておくと便利です。
- **CPU負荷の調整**: PF SLAM が重い場合は `particles` 数を減らすか、`threads` を CPU 核心数に合わせて設定してください。
- **地図の鮮明度**: `resolution` を小さくすると詳細な地図になりますが、メモリ消費と計算負荷が増加します。

## オフラインマッピング (rosbag)
`ros2 bag` を再生しながら地図を生成します。`launch/*_offline_launch.py` 内の `bag_file` パスを書き換えて使用してください。
```bash
ros2 launch iris_lama_ros2 slam2d_offline_launch.py
```

---
(C) 2019 Eurico Pedrosa, University of Aveiro.
Updated for ROS 2 Humble by Contributors.
