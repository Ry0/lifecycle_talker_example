# ROS2ライフサイクルノードのサンプル

このリポジトリはROS2の管理ライフサイクルノード（Managed Lifecycle Nodes）を使用した簡単なサンプルです。
このサンプルには、メッセージをPublishする`lifecycle_talker`ノード、標準的なSubscriberノード、コマンドラインツールとLaunchファイルを使用してライフサイクルを制御する例が含まれています。

## 概要

ROS2ライフサイクルノードは、ノードの状態管理に標準化されたアプローチを提供します。ノードは以下のような異なる状態を遷移します：
- Unconfigured
- Inactive
- Active
- Finalized

このサンプルでは以下を示しています：

- パラメータ処理を含む基本的なライフサイクルPublisherノードの実装
- メッセージをリッスンする通常のSubscriberノード
- コマンドラインツールを使用したライフサイクルの制御方法
- 自動および手動のライフサイクル制御のためのLaunchファイル

## ディレクトリ構造

```
lifecycle_talker_example/
├── launch/
│   ├── lifecycle_auto.launch.py    # 自動状態遷移のlaunchファイル
│   └── lifecycle_manual.launch.py  # 手動制御用のlaunchファイル
├── lifecycle_talker_example/
│   ├── __init__.py
│   ├── lifecycle_talker.py         # ライフサイクルノード実装
│   └── lifecycle_subscriber.py     # 通常のSubscriber
├── package.xml
├── resource/
│   └── lifecycle_talker_example
└── setup.py
```

## インストール方法

```bash
# ワークスペースに移動
cd ~/ros2_ws/src/
git clone git@github.com:Ry0/lifecycle_talker_example.git
```

```bash
# パッケージをビルド
cd ~/ros2_ws/
colcon build --symlink-install --packages-select lifecycle_talker_example
source install/setup.bash
```

## 使用方法

### 自動ライフサイクル制御（Launchファイル）

ライフサイクルノードを自動的に設定してアクティブ化するには：

```bash
ros2 launch lifecycle_talker_example lifecycle_auto.launch.py
```

このLaunchファイルは：
1. ライフサイクルノードを起動
2. 自動的に `configure` 状態遷移を実行
3. `configure` 完了後、自動的に `activate` 状態遷移を実行
4. ライフサイクルノードが完全にアクティブになった後、サブスクライバーを起動

### 手動ライフサイクル制御

ライフサイクルノードを手動で制御するには：

```bash
# 手動制御モードでノードを起動
ros2 launch lifecycle_talker_example lifecycle_manual.launch.py
```

または、

```bash
ros2 run lifecycle_talker_example lifecycle_talker 
```

次に、別のターミナルでライフサイクルを制御します：

```bash
# 現在の状態を確認
ros2 lifecycle get /my_lifecycle_node

# ノードを設定状態に遷移
ros2 lifecycle set /my_lifecycle_node configure

# ノードをアクティブ状態に遷移
ros2 lifecycle set /my_lifecycle_node activate

# パブリッシュされるメッセージを確認
ros2 topic echo /lifecycle_topic

# ノードを非アクティブ状態に遷移
ros2 lifecycle set /my_lifecycle_node deactivate

# ノードをクリーンアップ
ros2 lifecycle set /my_lifecycle_node cleanup

# ノードをシャットダウン
ros2 lifecycle set /my_lifecycle_node shutdown
```

### パラメータの設定
Unconfigured状態のときだけ、パラメータ変更を許容するような実装にしています。
(明示的にパラメータ変更を反映してもらうため)


#### Launchファイルでパラメータを指定

```bash
ros2 launch lifecycle_talker_example lifecycle_auto.launch.py \
  message:="カスタムメッセージ" publish_frequency:=3.0
```

#### コマンドライン

```bash
ros2 param set /my_lifecycle_node message "Hello from command line"
ros2 param get /my_lifecycle_node message
```

## ライフサイクルノードの実装

このサンプルのライフサイクルノードは以下のコールバックを実装しています：

- `on_configure`: パラメータを読み込み、リソースを初期化します
- `on_activate`: パブリッシャーをアクティブ化し、メッセージのパブリッシュを開始します
- `on_deactivate`: パブリッシュを停止し、パブリッシャーを非アクティブ化します
- `on_cleanup`: リソースを解放します
- `on_shutdown`: ノードをシャットダウンします(未実装)

## ライセンス
MIT
