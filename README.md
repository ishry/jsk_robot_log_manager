# jsk_robot_log_manager

- hrpsys，Neuro VM，ROSのログをまとめて管理するパッケージ
- neuro vmは複数個同時に管理できる

## 環境構築

### ソースコードの取得

catkin workspaceの `src` 以下へcloneしてbuildする．

```bash
cd ~/catkin_ws/src
git clone https://github.com/ishry/jsk_robot_log_manager.git
cd ..
catkin build jsk_robot_log_manager
source devel/setup.bash
```

### hrpsysの端末ログ

実機のbyobuから通常どおり起動すると，同じセッションの `hrpsys` ペインを監視する．
`log start`〜`log stop` の端末出力は試行フォルダの `hrpsys/console.log` に保存する．
byobu外から起動する場合は `hrpsys_console_pane:=%7` のようにペインIDを指定する
（実際のIDは `tmux list-panes -a -F '#{window_name} #{pane_id}'` で確認）．
すでに出力を保存する設定があるペインはその設定を維持し，監視を開始できない旨を表示する．
監視を起動しない場合は `record_hrpsys_console:=false` を指定する．

### Neuro VMの設定

[JskNeuroLogPlugin.hpp](include/jsk_robot_log_manager/JskNeuroLogPlugin.hpp)
をNeuro実装のincludeディレクトリへコピーし，次の2箇所を追加する．

```cpp
#include "JskNeuroLogPlugin.hpp"
```

```cpp
this->setPlugin(new JskNeuroLogPlugin({7001}));
```

Pluginの読み込みについて，同じNeuro実装から2プロセスを起動している場合は以下のように書く．この場合，自動的に，先に起動したプロセスが7001，次のプロセスが7002となる．

```cpp
this->setPlugin(new JskNeuroLogPlugin({7001, 7002}));
```

3プロセス以上の場合は候補ポートを増やす．

```cpp
this->setPlugin(new JskNeuroLogPlugin({7001, 7002, 7003, 7004}));
```

別のNeuro VMを起動する場合は，それぞれにPluginを導入して，ポートをそれぞれ設定する．

```cpp
// neuro_vm A
this->setPlugin(new JskNeuroLogPlugin({7001}));

// neuro_vm B
this->setPlugin(new JskNeuroLogPlugin({7002}));
```

ログマネージャとは別のPCでNeuro VMを起動する場合は，外部から接続できるアドレスで待ち受ける．

```cpp
this->setPlugin(new JskNeuroLogPlugin({7001}, "0.0.0.0"));
```

#### hrpsys・rosの設定

- 不要

#### jsk_robot_log_manager側の接続設定

使用するNeuro VMの名前，host，portをYAMLへ記述し，`config/default.yaml` を更新する．別のYAMLを使用する場合は，起動時に `config_file` を指定する．

```yaml
neuro_targets:
  - name: neuro_a
    host: 127.0.0.1
    port: 7001
    required: true
  - name: neuro_b
    host: 127.0.0.1
    port: 7002
    required: true
```

- `name`：保存ディレクトリ名にも使う一意な名前．
- `host`：Neuro VMのホスト名またはIPアドレス．ログマネージャと同じPCなら127.0.0.1で良い．異なるPCならそのPCのIPアドレスを記入する．
- `port`：`JskNeuroLogPlugin` に設定したポート．
- `required`：接続できない場合に記録全体を開始失敗にするか．
- `enabled`：任意項目．`false` にすると対象から除外する．

シミュレーションなどでNeuro VMを使用しない場合は`required: false` にする．Neuro VMへ接続できなくても，警告を表示してhrpsysとrosbagの記録を継続する．

Neuro VMが別PCにある場合は，ログマネージャとNeuro VMの両方から同じ絶対パスで参照できる共有ファイルシステムが必要となる．

## 使用方法

### 1．ログ管理ノードの起動

既存のROS masterへ接続する環境を設定し，別ターミナルで起動する．
このターミナルはログ取得中も起動したままにする．

```bash
source ~/catkin_ws/devel/setup.bash
rosset <ROS masterのホスト>

roslaunch jsk_robot_log_manager jsk_robot_log_manager.launch
```

任意の設定ファイルや保存先を指定する場合は，次のように起動する．

```bash
roslaunch jsk_robot_log_manager jsk_robot_log_manager.launch \
  config_file:=/absolute/path/to/log_config.yaml \
  log_root:=/absolute/path/to/logs
```

### 2．ログ取得の開始・確認・終了

別ターミナルから操作する．

```bash
source ~/catkin_ws/devel/setup.bash
rosset <ROS masterのホスト>

rosrun jsk_robot_log_manager jsk-robot-log start experiment --duration 180
rosrun jsk_robot_log_manager jsk-robot-log status
rosrun jsk_robot_log_manager jsk-robot-log stop
```
- `duration` は，hrpsysのリングバッファへ何秒分保持するかを指定する．記録時間が `duration` を超えると，hrpsysは直近の指定時間分だけを保存する．
- `start` を実行すると，hrpsysの `maxLength` と `clear` が呼ばれ，Neuro VMとrosbagを含むログの取得が開始される．`stop` を実行すると，Neuro VMの `log stop`，hrpsysの `save`，rosbagの終了が実行される．
- `stop` 後もログ管理ノードは動作し続けるため，再度 `start` できる．
- 全て終わったら，`roslaunch` を実行したターミナルでCtrl-Cする．

## その他

### ログの保存場所

デフォルトの保存先は `~/jsk_robot_logs` ．

```text
<日時>_<セッション名>/
  manifest.json
  hrpsys/
  neuro/
    <Neuro名>/
  ros/
```

`manifest.json` には，各記録元の状態，保存ファイル数，警告，エラーを記録する．

### サービスとnamespace

デフォルトでは，次のprivate serviceとtopicを作成する．

```text
/jsk_robot_log_manager/start
/jsk_robot_log_manager/stop
/jsk_robot_log_manager/status
/jsk_robot_log_manager/event
```

ノード名やnamespaceを変えた場合は，CLIの `--manager` を指定する．

```bash
roslaunch jsk_robot_log_manager jsk_robot_log_manager.launch \
  namespace:=robot_a node_name:=log_manager

rosrun jsk_robot_log_manager jsk-robot-log \
  --manager /robot_a/log_manager start experiment
```

### 体内PCのlogを手元PCにコピー
- 手元PCで以下を実行
```bash
scp -r <user>@<体内PC>:/home/<user>/jsk_robot_logs/<セッション名> <path to logdir>
```
