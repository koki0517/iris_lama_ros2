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

## 各ノードの詳細とパラメータ

### 1. Online SLAM (`slam2d_ros`) / PF SLAM (`pf_slam2d_ros`)
逐次的にマップを生成する SLAM ノードです。

**起動方法:**
```bash
# 標準版 SLAM
ros2 launch iris_lama_ros2 slam2d_live_launch.py
# パーティクルフィルタ版 SLAM
ros2 launch iris_lama_ros2 pf_slam2d_live_launch.py
```

**共通主要パラメータ:**
- `global_frame_id` (string, default: "map"): マップフレームID。
- `odom_frame_id` (string, default: "odom"): オドメトリフレームID。
- `base_frame_id` (string, default: "base_link"): ロボット本体のフレームID。
- `scan_topic` (string, default: "/scan"): 購読するレーザースキャントピック。
- `initial_pos_x/y/a` (double, default: 0.0): 起動時の初期位置（x, y [m]）と向き（a [rad]）。
- `resolution` (double, default: 0.05): 生成するマップの解像度 [m/cell]。
- `d_thresh` (double): 地図更新を行う最小移動距離 [m]。 (標準版: 0.01, PF版: 0.5)
- `a_thresh` (double, default: 0.25): 地図更新を行う最小回転角度 [rad]。
- `map_publish_period` (double, default: 5.0): `/map` トピックに最新地図を配信する周期 [秒]。
- `mrange` (double, default: 16.0): レーザースキャンの最大有効距離 [m]。

**PF SLAM 固有パラメータ:**
- `particles` (int, default: 30): パーティクル数。
- `threads` (int, default: -1): 使用スレッド数（-1: 無効, 0: 自動設定）。

---

### 2. 2D Localization (`loc2d_ros`)
既存の地図を使用して自己位置推定（map -> odom の TF 配信）を行います。

**起動方法:**
```bash
ros2 launch iris_lama_ros2 loc2d_launch.py
```

**自己位置推定の仕組み:**
- 起動直後、パラメータ `initial_pos_x/y/a` で指定された位置を起点として推定を開始します。
- 実際のロボットの位置がズレている場合は、**RViz2 の "2D Pose Estimate" ボタン** を使用して地図上の正しい位置を指定してください。トピック `/initialpose` を受信すると、推定状態がその位置へリセットされます。
- 推定が動作するためには、外部（`nav2_map_server` など）が `/map` サービスを提供している必要があります。

**主要パラメータ:**
- `global_frame_id / odom_frame_id / base_frame_id`: SLAMと共通。
- `scan_topic`: 購読するレーザースキャントピック。
- `initial_pos_x/y/a` (double, default: 0.0): 自己位置推定の開始位置。
- `d_thresh` (double, default: 0.01): 自己位置の再計算を行う移動距離しきい値 [m]。
- `a_thresh` (double, default: 0.2): 自己位置の再計算を行う回転角度しきい値 [rad]。
- `l2_max` (double, default: 0.5): ユークリッド距離マップで使用する最大距離 [m]。
- `strategy` (string, default: "gn"): スキャンマッチングの最適化戦略（"gn": Gauss-Newton, "lm": Levenberg-Marquardt）。
- `transform_tolerance` (double, default: 0.1): 配信する TF の有効期限（タイムスタンプへの加算値）[秒]。

---

## 運用のアドバイス
- **TFツリー**: 本ノードは `map -> odom` の変換を配信します。オドメトリ源（`odom -> base_link`）は別途必要です。
- **LiDARのフレーム名**: LiDARのフレーム名（`laser_link` など）は、受信する `/scan` メッセージのヘッダから自動取得されます。ベースフレームとの位置関係は TF から自動解決されるため、個別のパラメータ設定は不要です。
- **計算負荷**: 地図の解像度 (`resolution`) を上げたり、PFのパーティクル数 (`particles`) を増やすと精度が上がりますが、CPU負荷が増大します。

---
(C) 2019 Eurico Pedrosa, University of Aveiro.
Updated for ROS 2 Humble by Contributors.
