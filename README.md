# ROS 2＋Gazebo版：視覚認識つき探索ロボット v3

**対象：Apple Silicon Mac＋Docker Desktop／ROS 2 Humble／Gazebo Fortress／Mac側のOllama**

Webots版v2の「赤・青の箱をカメラで探し、VLMが確認した対象へ接近して停止する」をROS 2とGazeboに移植したプロジェクトです。**Webotsは使いません。以前の版は削除せず、別フォルダとして展開してください。**

> **検証範囲**：作成環境で純粋Python、模擬ROS、模擬HTTPサーバ、設定整合性の合計80テストを実行しました。**Dockerイメージのビルド、実ROS 2 DDS通信、Gazeboの物理シミュレーション、実Ollama推論、M5 Pro上の統合動作は未実行です。** 統合診断コマンドは、その部分を手元で確認するために含めています。テスト結果は `docs/validation.md` を参照してください。

## 1. 何が変わったか

| 項目 | v2 | 今回のv3 |
|---|---|---|
| シミュレータ | Webots | Gazebo Fortress（Ignition Gazebo 6） |
| ロボット | e-puck2 | 自作の小型差動二輪 `visual_bot` |
| カメラ | Webots Camera API | `/camera/image_raw`、sensor_msgs/Image |
| 距離 | e-puck固有ToF・赤外線 | `/scan`、sensor_msgs/LaserScan、360度 |
| 運動指令 | 左右車輪の角速度 | `/cmd_vel`、geometry_msgs/Twist |
| ROS 2 | 未使用 | 制御ノード＋独立した指令監視ノード＋bridge |
| AI | Ollama＋Qwen2.5-VL 3B | 同じモデルをMac側で再利用 |
| AI応答待ち | 同期呼び出し | 別スレッドで推論し、待機中は停止指令を継続 |
| 指示変更 | JSONファイル＋Reset | 起動中に `/mission` を送る |
| アセット | Webots標準アセット | 同梱SDFの基本形状のみ。Fuel参照なし |

**Gazebo Classic 11用ではありません。Humble＋Fortressの組み合わせです。Jazzy＋Harmonic用のコマンドやプラグイン名と混在させないでください。**

AIの役割はv2と同様、自然言語の目標色抽出とカメラ画像内の対象確認です。探索・追跡・車輪指令は従来型の制御です。VLA、強化学習、実行時のコード生成、AIによる連続速度計画は追加していません。

```text
Mac
  Ollama + qwen2.5vl:3b（既存モデル）
    ↑ 127.0.0.1:11434
  認証付きローカル中継（tools/ollama_proxy.py）
    ↑ Dockerから host.docker.internal:11435
  Docker Desktop / Ubuntu 22.04
    ├─ Gazebo Fortress（物理・仮想カメラ・LiDAR）
    ├─ ros_gz_bridge（Gazebo Transport ⇄ ROS 2）
    ├─ visual_robot（画像認識・追跡・状態機械）
    ├─ velocity_guard（指令・距離・時刻の監視）
    └─ noVNC（Macのブラウザにデスクトップを表示）
```

ROS 2ノードとGazeboは同じコンテナ内です。Macとの間にDDSネットワークを構築する必要はありません。AIはHTTP経由で呼び出します。

## 2. Mac側で準備するもの

Docker Desktop、Ollama、Python 3が必要です。以前導入したPython 3.12とOllamaを流用できます。**MacへROSやGazeboをbrewで導入する必要はありません。コンテナ内のPythonはUbuntu 22.04の3.10です。**

```bash
open -a Docker
open -a Ollama
docker info
docker compose version
ollama list
curl -f http://127.0.0.1:11434/api/tags
```

`qwen2.5vl:3b` が一覧にあればモデルの再取得は不要です。ない場合だけ：

```bash
ollama pull qwen2.5vl:3b
```

Docker Desktopへのメモリ割当は12〜16GB程度を試す目安とし、Mac側Ollama用にもメモリを残してください。これは実測の最低要件ではありません。イメージとビルドキャッシュのための空き容量も必要です。サイズ・所要時間はキャッシュと取得時のパッケージ次第です。

Docker Desktopが未導入の場合は、[公式のMac用インストール手順](https://docs.docker.com/desktop/setup/install/mac-install/)でApple Silicon用を選んでください。利用条件の確認と初回起動はご自身で行ってください。
## 3. ZIP展開・初回ビルド

```bash
cd ~/Downloads
unzip physical_ai_ros2_gazebo_v3.zip
cd physical_ai_ros2_gazebo_v3
chmod +x run.sh
./run.sh init
./run.sh build
./run.sh up
```

`build` は初回にROS、Gazebo、描画ライブラリ、noVNCなどを取得するためネット接続が必要です。エラーになったら、そのまま次に進まずログを確認してください。

`up` は既存のイメージを利用し、**再ビルド・自動pullは行いません**。日々の起動は `./run.sh up` です。

ブラウザのアドレス欄に入力：

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

デスクトップとターミナルが表示されれば、表示基盤の準備完了です。**Gazeboはまだ起動していません。**

標準ではCPUアーキテクチャを強制しないため、Apple SiliconではARM64を使います。GazeboのARM対応はbest-effortであり、手元での動作確認が必要です。[S2] AMD64エミュレーション用の予備設定も同梱しています。

## 4. Mac側のAI中継を起動（ターミナルA）

**Macのターミナル**で、このフォルダから実行します。

```bash
./run.sh proxy
```

このターミナルは開いたままにします。終了は `Ctrl+C` です。

Docker内の `127.0.0.1` はMacではなくコンテナ自身を指すため、中継経由でMacのOllamaを呼びます。Ollamaを無認証でLANへ公開する代わりに、`.env` に生成したランダムなトークンを必須とし、`GET /api/tags` と指定モデルの `POST /api/chat` だけを転送します。モデルの取得・削除、任意URLへの転送は許可しません。

**注意**：中継はDockerから接続できるよう既定で `0.0.0.0:11435` に待ち受けます。トークン認証はありますがHTTPでありTLS暗号化はありません。信頼できるローカルネットワークで使い、不要時は止めてください。`.env` をGitHubやレポートに含めないでください。ファイアウォール全体を無効化する必要はありません。

## 5. 診断・Gazebo起動（ターミナルB）

別のMacのターミナルで同じフォルダから：

```bash
./run.sh doctor
```

ROSパッケージ、Gazeboのバージョン、OpenGL、コンテナからOllamaへの接続、モデルの有無を確認します。`FAIL` の項目を修正してから進めます。診断は追加インストールをしません。

```bash
./run.sh sim
```

ターミナルBを開いたままにします。ブラウザのデスクトップ内でGazeboが起動します。シミュレーション時間は進みますが、制御ノードは **IDLE（指示待ち）** です。

### センサ受信を先に確認（ターミナルC）

```bash
./run.sh check
```

カメラ・LiDAR・進んでいるシミュレーション時刻・オドメトリの受信を確認し、実際に届いたカメラ画像を `logs/smoke_camera.png` に保存します。Finderなどで開いて箱と床が写っているか見てください。`PASS` はセンサ診断であり、AIタスク成功ではありません。

映像を連続表示する場合：

```bash
./run.sh camera
```

表示される `rqt_image_view` で `/camera/image_raw` を選択してください。

## 6. ミッションを実行（ターミナルC）

```bash
./run.sh mission "赤い箱を探して近づいて停止してください"
```

Ollamaが指示を解析して `/mission` を送ると、探索を始めます。設計上の流れは次のとおりです。

```text
SEARCH → SETTLE → VERIFY → APPROACH → DONE
                           ↓ロスト
                         SEARCH
```

`SETTLE` では旋回を止め、停止後の新しい画像を取得します。`VERIFY` ではVLMへ画像を送信し、応答待ち中もROSの制御ループから速度0を送り続けます。

AIが対象を確認すると、画像の色領域位置とLiDARに基づいて接近します。AI通信失敗は停止し、**勝手にAIなし制御へ切り替わりません**。

青い箱への変更：

```bash
./run.sh mission "青い箱を探して、その近くで止まって"
```

古いミッションの推論が後から完了しても、新しいミッションの承認には使わないようにしています。

自然言語解析だけを省く診断：

```bash
./run.sh mission --manual red
```

これは目標色の設定だけが手動です。通常起動の `vision_mode=ai` では **走行中の画像VLMは引き続き必要** です。

### 停止

```bash
./run.sh halt
```

停止して処理中のAI結果も無効化します。再開は新しいミッション送信です。

全体を終える場合はターミナルBで `Ctrl+C`、続けて：

```bash
./run.sh down
```

コンテナを停止・削除しますが、Mac側の `src/`・`logs/` とビルド済みイメージは残ります。ターミナルAの中継も `Ctrl+C` で終了します。

## 7. 統合試験を分けて行う

**センサ受信**：`./run.sh check`。

**本物のVLMへの入力確認**：ロボットのミッションを始める前に：

```bash
./run.sh check --vision
```

受信したカメラ画像をOllamaへ送り、返答を `logs/smoke_vision.json` に保存します。AIの返答と画像が一致するかは目視で確認してください。

**ROS指令からの移動確認**：通常の `sim` を `Ctrl+C` で終了し、ターミナルBで：

```bash
./run.sh sim controller:=false
```

ターミナルCで：

```bash
./run.sh check --move
```

低速前進を行い、オドメトリの変化を確認します。通常制御との二重指令を避けるため、`visual_robot` が動いている場合は検査を拒否します。初期ワールドの配置で実施してください。その後は再び停止し、`./run.sh sim` で通常起動へ戻します。

**タスク成功の確認**：赤・青、配置変更、ロスト、障害物の条件でミッションを実行し、ログと動画を残します。課題を達成したと説明するのはこの確認の後です。

## 8. オフライン利用

ネット取得が必要なのは、Docker Desktop・Ollama・ホストPythonの導入、`./run.sh build`、モデル取得、このZIPの保存です。

**ワールドにはFuelモデル・メッシュ・外部テクスチャの参照がありません。床・箱・壁・ロボットはSDFで定義済みです。Webots用assetsも不要です。**

準備後の `up`・`proxy`・`doctor`・`sim`・`mission` は追加ダウンロード処理を行いません。DockerとMacのローカル通信は必要です。ソフトの自動更新などは別の話です。オフラインでセンサ受信と推論まで確認してから持ち出してください。提出先へのアップロードにはネット接続が必要です。

ビルド済みイメージを保存する場合：

```bash
docker image save pai-ros2-gazebo:v3 -o "$HOME/Downloads/pai_ros2_gazebo_v3_image.tar"
```

復元：

```bash
docker image load -i "$HOME/Downloads/pai_ros2_gazebo_v3_image.tar"
```

Ollamaモデルはこのtarに含まれず、Mac側で別管理です。またARM64のイメージをAMD64用として扱うことはできません。

## 9. よくある問題

| 症状 | 確認すること |
|---|---|
| Docker daemonへ接続不可 | MacでDocker Desktopを起動し `docker info` を確認 |
| 6080番ポート競合 | `docker ps` で既存コンテナを確認。無関係なものを強制停止しない。composeの公開ポートを6081へ変える方法もある |
| AI中継401 | `.env` のトークンとコンテナが不一致。中継を再起動し `down` → `up` |
| AI中継502 | Mac側Ollamaの停止または推論タイムアウト |
| 中継へ接続拒否 | `proxy` がMacで動いているか、11435番競合、ファイアウォールを確認 |
| モデルがない | Mac側 `ollama list` を確認。Docker側へ重複取得しない |
| WAIT_CLOCK | GazeboのPause、/clockのbridge、再生状態を確認 |
| WAIT_SENSORS | /camera/image_raw、/scan、描画系のログを確認 |
| VLMが対象なしと答える | logs内PNGを見て、画像取得の問題と推論の問題を切り分ける |

### 描画が真っ黒／センサが来ない

このDocker設定は **Mesaソフトウェア描画** です。Apple GPUをコンテナへ直接渡す前提ではありません。`doctor` のOpenGL結果と `logs/xvfb.log`、起動時エラーを確認します。

GUIを省いて確認：

```bash
./run.sh sim gui:=false
```

**GUIなしでもカメラとgpu_lidarのレンダラは必要です。** headlessにするだけで画像生成の要件がなくなるわけではありません。

Ogre 2由来のエラーなら、一度停止してOgre 1も試せます。[S5]

```bash
./run.sh sim render_engine:=ogre
```

センサ側とGUI側を両方切り替えます。切替後も `check` で実画像とLiDARを検査してください。センサが欠けたまま進めないでください。`--privileged` や `seccomp=unconfined` は標準設定に含めていません。

### ARM64パッケージでビルドできない

エラー全文を保存し、依存・リポジトリの問題かを確認します。AMD64を試す場合のみ `.env` に追記：

```text
PAI_PLATFORM=linux/amd64
```

```bash
./run.sh down
./run.sh build
./run.sh up
```

Apple Siliconではエミュレーションになり、速度低下や別の互換性問題があり得ます。成功保証ではありません。戻す場合は指定を削除して再ビルドします。アーキテクチャ変更には新しいイメージの構築が必要です。

### AIを使わない診断モード

Ollamaから独立してROS/Gazeboを調べる場合：

```bash
./run.sh sim vision_mode:=rules
```

別ターミナルで `./run.sh mission --manual red`。これは画像AIを使わず、ログにもrulesと表示されます。**AI活用の実証として使わないでください。** 通常モードから自動的にこのモードになることはありません。

## 10. 開発・記録

```bash
./run.sh test
./run.sh shell
./run.sh record
```

`record` はrosbag2記録です。動画ファイルではありません。動画提出には別途画面録画を使います。

`src/` はMac側からマウントします。Python編集後は通常、simの再起動で反映されます。エントリポイント等を変更した場合：

```bash
./run.sh rebuild
```

新しい依存パッケージを導入するならDockerfileを変更して `build` が必要です。SDF変更は次回のシミュレータ起動で読みます。`tools/generate_world.py` は初期ワールド再生成用なので、実行すると手作業のSDF編集を上書きします。

## 11. 限界と提出時の注意

- 停止監視は学習用です。安全認証済み実機用コントローラではありません。
- 回避は局所的な後退・旋回・前進であり、袋小路等を解く保証はありません。
- 到達判定は画像上の大きさ・中心位置と前方LiDARの組合せです。LiDARが目標の箱を測っているか、物体IDで厳密に対応づけていません。別の障害物が一直線上にある場合の誤判定などを検証してください。
- VLM確認と色追跡は厳密な物体グラウンディングではありません。同じ色の別物がある場合の限界が残ります。
- 制御ノードは正解の箱座標もオドメトリも購読しません。odomは診断・記録専用です。
- SLAM、Nav2、把持、運搬はありません。
- 実際の描画速度・AI精度・タスク成功率は未測定です。
- `report_template.md` に、自分で加えた工夫と実測結果を記入してください。付属コードを使っただけで修了が保証されるわけではありません。
- `.env`、個人情報、秘密情報を公開しないでください。課題資料そのものはこのZIPに転載していません。

## 12. 公開資料

[S1] ROS/Gazeboの対応：https://gazebosim.org/docs/fortress/ros_installation/

[S2] Fortress対応環境：https://gazebosim.org/docs/fortress/install/

[S3] Humble bridge：https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md

[S4] センサ：https://gazebosim.org/docs/fortress/sensors/

[S5] 描画トラブル：https://gazebosim.org/docs/fortress/troubleshooting/

[S6] Dockerネットワーク：https://docs.docker.com/desktop/features/networking/

[S7] Ollama設定：https://docs.ollama.com/faq

[S8] 差動二輪プラグイン：https://gazebosim.org/docs/fortress/moving_robot/

参照日：2026-10-19。