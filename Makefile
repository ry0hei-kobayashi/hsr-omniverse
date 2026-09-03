# ============================================================================
# carrobo-isaac Makefile (ロボット実習用 Isaac Sim 環境)
#
# 基本の使い方:
#   make build      イメージをビルド (初回のみ。30分〜1時間程度)
#   make build nocache  キャッシュを使わず全部焼き直す (依存を取り直したいとき)
#   make up         シミュレータ一式を起動 (初回はシェーダー生成で10〜20分)
#   make down       停止・コンテナ削除
#   make exec ros2      ros2 コンテナに入る (ros2 topic list などを叩く場所)
#   make exec isaacsim  isaacsim コンテナに入る
#   make logs       全サービスのログを表示
#   make ps         コンテナの状態を表示
#
# 競技モード:
#   make up TIME=600   競技時間 600 秒 (シミュレータ内時間)。4方向の観戦カメラで
#                      録画し、時間が来たら recordings/ に mp4 を保存して自動終了。
#
# カメラ調整モード:
#   make tune       観戦カメラ4台だけ作って起動 (録画なし)。GUI で位置・画角を
#                   動かすと recordings/tune/ に設定 yaml とプレビュー画像が出る。
#
# 把持のしかた:
#   make up GRASP=1     アタッチ把持 (物体が指に吸い付く) を有効にする。
#                       既定は無効で、指と物体の物理接触 (摩擦) だけで掴む。
#                       物理挙動として素直だが、摩擦や把持幅の条件が
#                       合わないと滑って落ちる。確実に保持させたいとき
#                       だけ GRASP=1 を使う。
#
# DDS を loopback に閉じるモード (LANへのマルチキャスト漏洩対策):
#   make up localhost   assets/cyclonedds.localhost.xml を使って起動する。
#                       同一ホスト内だけで ROS 通信する場合はこちらを推奨。
#                       詳細は docs/dds-localhost.md
#
# 開発モード (シミュレータの Python を手動実行したいとき):
#   make dev up     コンテナだけバックグラウンド起動 (シミュレータは自動起動しない)
#   make dev run    同じ端末でシミュレータを実行 (ログ・エラーがここに出る)
#   make dev down   停止
# ============================================================================

# -p carrobo-isaac: compose のプロジェクト名。これを付けないとフォルダ名 (env_docker) が
# プロジェクト名になり、hsr-omniverse のイメージ (env_docker-isaacsim 等) と衝突・上書きしてしまう。
COMPOSE := docker compose -p carrobo-isaac -f env_docker/docker-compose.yml

# --- dev modifier -----------------------------------------------------------
# 'dev' を付けると docker-compose.dev.yml を重ねて適用する。
# isaacsim は 'sleep infinity' で待機起動し、Python は自動実行されない。
# その後 'make dev run' で手動実行する (ログがその端末だけに出る)。
DEV := $(if $(filter dev,$(MAKECMDGOALS)),1,)
ifeq ($(DEV),1)
  COMPOSE := $(COMPOSE) -f docker-compose.dev.yml
  # dev では scripts/ をディレクトリごと live マウントするので、そちらを実行する
  # (編集がコンテナ再起動なしで反映される)。
  LAUNCH_PY := /app/scripts/launch_isaacsim.py
  # dev では up をバックグラウンド(-d)にし、端末をすぐ返す。
  UP_FLAGS := -d
else
  # 通常起動はフォアグラウンド (全コンテナのログを表示)。
  LAUNCH_PY := /app/launch_isaacsim.py
  UP_FLAGS :=
  # 競技モード (TIME=秒) のときは、シミュレータが自動終了したら
  # 他のコンテナ (ros2 等) も一緒に止めて make up 自体を終わらせる。
  ifneq ($(TIME),)
    UP_FLAGS += --abort-on-container-exit
  endif
endif

# --- localhost modifier -----------------------------------------------------
# 'localhost' を付けると DDS を loopback (lo) に閉じたプロファイルで起動する。
# 点群・画像を物理NICからマルチキャストで撒かなくなる (理由: docs/dds-localhost.md)。
#   make up localhost
# 既定 (無指定) は従来どおり assets/cyclonedds.xml (マルチキャスト版)。
LOCALHOST := $(if $(filter localhost,$(MAKECMDGOALS)),1,)
ifeq ($(LOCALHOST),1)
  # 'localhost' と明示された以上、環境変数より優先させる (:= で上書き)。
  CYCLONEDDS_URI := file:///cyclonedds.localhost.xml
else
  CYCLONEDDS_URI ?= file:///cyclonedds.xml
endif
# Makefile 内で定義した変数は自動では export されず、compose の
# ${CYCLONEDDS_URI} 展開に届かないので明示的に export する。
export CYCLONEDDS_URI

# --- competition seed selection ---------------------------------------------
# 固定済みの競技Seed 1〜4を選ぶ。ユーザー向けには指定例どおり小文字を使えるようにし、
# CI等では COMPE_SEED=1 も受け付ける。
#   make up localhost compe_seed=1
COMPE_SEED ?= $(compe_seed)
ifneq ($(strip $(COMPE_SEED)),)
  ifeq ($(filter 1 2 3 4,$(COMPE_SEED)),)
    $(error compe_seed は 1, 2, 3, 4 のいずれかを指定してください: '$(COMPE_SEED)')
  endif
  OBJECT_PLACEMENT_CONFIG := /app/configs/placement.compe_seed$(COMPE_SEED).yaml
else
  OBJECT_PLACEMENT_CONFIG :=
endif
# --- nocache modifier -------------------------------------------------------
# 'nocache' を付けるとレイヤキャッシュを一切使わずに焼き直す。
#   make build nocache
# 依存 (apt/pip/git) を最新で取り直したいとき、キャッシュが壊れている疑いがあるとき用。
# isaacsim イメージは 18GB あるので数十分〜1時間以上かかる。通常は付けないこと。
# 環境変数派のために NO_CACHE=1 でも同じ意味にする。
# ※ --pull は付けない。ベースが nvcr.io/nvidia/isaac-sim:4.5.0 (NGC 認証が要る) なので、
#    ログインしていない環境では pull 失敗でビルドごと落ちる。ベースまで取り直したいときだけ
#    手で `make build nocache PULL=--pull`。
PULL ?=
NO_CACHE ?= $(if $(filter nocache,$(MAKECMDGOALS)),1,0)
ifeq ($(NO_CACHE),1)
  BUILD_FLAGS := --no-cache $(PULL)
else
  BUILD_FLAGS :=
endif

# Isaac 側と Apptainer 側で必ず一致させる値。make up ROS_DOMAIN_ID=31 で変えられる。
ROS_DOMAIN_ID ?= 26
export ROS_DOMAIN_ID

# 検証用にコンテナ内パス -> ホスト上の assets/ のファイル名へ戻す
DDS_FILE := $(patsubst file:///%,%,$(CYCLONEDDS_URI))

# --- Targets ----------------------------------------------------------------
.PHONY: help dev localhost nocache dds-check build up down logs ps exec ros2 isaacsim run tune tune-apply
.DEFAULT_GOAL := help

# RViz2 は既定でオフ。見たいときだけ `make up RVIZ=1`。
RVIZ ?= 0
USE_RVIZ := $(if $(filter 1 true TRUE yes YES on ON,$(RVIZ)),true,false)
BASE_DIRECT_DRIVE ?= 1
# 直接駆動の台車を「衝突形状を持たない kinematic anchor」に溶接して毎ステップ
# テレポートするか (1)、平面ジョイント上の動剛体として速度を書くか (0)。
# 0 は速度書き込みと D6 位置ドライブの二重制御になり、拘束ソルバー内で競合して
# 追従が崩れる (純回転で並進が 3 倍、0.5m 指令で 0.12m しか進まない等、実測は
# scripts/hsr.py のコメント参照)。既定は 1。
# 1 は本来「衝突フィードバックが無い」「走行中に arm_flex が振られる」という
# 欠点があったが、どちらも実装で解消した:
#   - 壁: anchor を進める前に PhysX の球スイープでクランプする
#         (BASE_KINEMATIC_COLLISION / BASE_COLLISION_*)
#   - 腕: anchor を kinematic target で動かし、PhysX 自身に速度を導出させる
#         (BASE_KINEMATIC_ANCHOR_MODE=usd)
# 切り分けたいときだけ `make up BASE_KINEMATIC_DRIVE=0` で従来経路へ戻せる。
BASE_KINEMATIC_DRIVE ?= 1
# kinematic 駆動時の壁クランプ。0 にすると台車が壁をすり抜ける。
BASE_KINEMATIC_COLLISION ?= 1
# クランプに使う台車の外周半径 [m] / スイープ球の高さ [m] / 手前に残す余裕 [m]。
BASE_COLLISION_RADIUS ?= 0.24
BASE_COLLISION_HEIGHT ?= 0.25
BASE_COLLISION_MARGIN ?= 0.01
# kinematic anchor の動かし方。usd = USD の xform を書いて PhysX に kinematic
# target として解釈させる (anchor が速度を持つので articulation が滑らかに運ばれ、
# arm_flex が叩かれない)。dc = dc.set_rigid_body_pose で瞬間移動 (従来動作)。
BASE_KINEMATIC_ANCHOR_MODE ?= usd
# arm_flex のドライブ力上限 [N*m]。台車搬送時の外乱に負けて落ちるなら上げる。
ARM_FLEX_MAX_FORCE ?= 300
# --- 引き出し (Room SW の trofast) ------------------------------------------
# 段違い棚と trofast を直動ジョイントで繋いで「本物の引き出し」にする (既定 1)。
# 0 にすると従来の「棚に置いてあるだけの箱」に戻る。
DRAWER_JOINTS ?= 1
# 引き出せる距離 [m]。箱の奥行きは 0.4235 m。
DRAWER_PULL_LIMIT ?= 0.30
# 引き心地 (粘性抵抗) [N/(m/s)]。重ければ下げる、勢い余るなら上げる。
DRAWER_DAMPING ?= 15.0
DRAWER_DAMPING_MAX_FORCE ?= 200.0
BASE_VELOCITY_WRITE ?= 1
BASE_TRAJ_P_GAIN ?= 2.0
BASE_TRAJ_D_GAIN ?= 0.5
BASE_TRAJ_I_GAIN ?= 4.0
BASE_TRAJ_I_LINEAR_LIMIT ?= 0.08
BASE_TRAJ_I_ANGULAR_LIMIT ?= 0.15
BASE_CMD_TAU ?= 0.10
BASE_WHEEL_ACCEL_LIMIT ?= 41.7
BASE_STEER_ACCEL_LIMIT ?= 5.0
# 台車の並進/回転の加速度上限 [m/s^2] / [rad/s^2]。
# reset_world 後のステップ応答を実測すると立ち上がりはこの値そのままの直線になり、
# BASE_CMD_TAU ではなくここが応答を支配する。0.35/0.8 では pumas_nav2 の経路追従が
# 曲がりきれず、ゴール手前の減速中に simple_move の attempts (wall tick 予算) が
# 尽きて abort していたため引き上げた。
BASE_LINEAR_ACCEL_LIMIT ?= 1.0
BASE_ANGULAR_ACCEL_LIMIT ?= 2.0
BASE_WHEEL_DRIVE_DAMPING ?= 10.0
# キャスターの転がり軸の粘性摩擦 (0 だと静止後も回り続ける)
BASE_PASSIVE_WHEEL_DAMPING ?= 0.001
BASE_WHEEL_DRIVE_MAX_FORCE ?= 10.0
BASE_STEER_DRIVE_DAMPING ?= 5.0
BASE_STEER_DRIVE_MAX_FORCE ?= 5.0
BASE_DIRECT_JOINT_DAMPING ?= 1.0
BASE_DIRECT_JOINT_MAX_FORCE ?= 1.0
BASE_DIRECT_ROOT_DAMPING ?= 2.0
BASE_BRAKE ?= 1
BASE_BRAKE_ENGAGE_LINEAR ?= 0.05
BASE_BRAKE_ENGAGE_ANGULAR ?= 0.10
BASE_BRAKE_CREEP_SPEED ?= 0.05
BASE_BRAKE_CREEP_ANGULAR ?= 0.15
BASE_BRAKE_K ?= 200000
BASE_BRAKE_C ?= 20000
BASE_BRAKE_MAX_FORCE ?= 20000
BASE_BRAKE_ANGULAR_K ?= 50000
BASE_BRAKE_ANGULAR_C ?= 5000
BASE_BRAKE_MAX_TORQUE ?= 5000
BASE_SETPOINT_MAX_LAG ?= 0.02
BASE_SETPOINT_MAX_LAG_ANGULAR ?= 0.05
BASE_GOAL_VELOCITY_TOLERANCE ?= 0.10
BASE_JOINT_BRAKE ?= 1
BASE_DIAG ?= 0

# fixed by ry0hei.k
# カメラ publish [Hz] = 60 / RENDER_EVERY_N_STEPS / (CAMERA_FRAME_SKIP + 1) * RTF
# 既定 60/2/1 = 30 Hz (実機の HSR と同等)。物理と whole-body 制御は常に 60 Hz。
# RTF が 1.0 を保てないときは CAMERA_FRAME_SKIP=1 (15 Hz) に落とす。
CAMERA_FRAME_SKIP ?= 0
RENDER_EVERY_N_STEPS ?= 2

# head_l/head_r ステレオ (1280x960 x2) は描画予算の 73% を食うのに購読者が居ないので既定 off。
# ステレオ画像が要るタスクのときだけ `make up ENABLE_STEREO_CAMERAS=1`。
ENABLE_STEREO_CAMERAS ?= 0

# 1 にすると観戦カメラの調整モード (make tune で自動的に 1 になる)
CAMERA_TUNE ?=

# reset_world の姿勢キャプチャをスポーン root の「サブツリー全体」に広げる (既定 1)。
# 天板のようなモデル内部のメッシュをギズモで動かしても reset で戻せるようになる。
# 走査は起動時の 1 回だけなので、そのコストが問題になったときだけ 0 にして
# root のみの旧挙動へ戻す (実測値は起動ログの [reset_world] captured ... に出る)。
RESET_DEEP_CAPTURE ?= 1

# isaacsim コンテナへ渡す環境変数の一覧。up (compose 用の変数代入) と
# run (docker compose exec の -e) で同じ一覧から生成し、片方だけ追加し忘れる
# のと、`-e -e` のような取りこぼしが起きないようにする。
SIM_ENV_NAMES = \
  OBJECT_PLACEMENT_CONFIG \
	BASE_DIRECT_DRIVE \
	BASE_KINEMATIC_DRIVE \
	BASE_KINEMATIC_COLLISION \
	BASE_COLLISION_RADIUS \
	BASE_COLLISION_HEIGHT \
	BASE_COLLISION_MARGIN \
	BASE_KINEMATIC_ANCHOR_MODE \
	ARM_FLEX_MAX_FORCE \
	DRAWER_JOINTS \
	DRAWER_PULL_LIMIT \
	DRAWER_DAMPING \
	DRAWER_DAMPING_MAX_FORCE \
	BASE_VELOCITY_WRITE \
	BASE_TRAJ_P_GAIN \
	BASE_TRAJ_D_GAIN \
	BASE_TRAJ_I_GAIN \
	BASE_TRAJ_I_LINEAR_LIMIT \
	BASE_TRAJ_I_ANGULAR_LIMIT \
	BASE_CMD_TAU \
	BASE_WHEEL_ACCEL_LIMIT \
	BASE_STEER_ACCEL_LIMIT \
	BASE_LINEAR_ACCEL_LIMIT \
	BASE_ANGULAR_ACCEL_LIMIT \
	BASE_WHEEL_DRIVE_DAMPING \
	BASE_PASSIVE_WHEEL_DAMPING \
	BASE_WHEEL_DRIVE_MAX_FORCE \
	BASE_STEER_DRIVE_DAMPING \
	BASE_STEER_DRIVE_MAX_FORCE \
	BASE_DIRECT_JOINT_DAMPING \
	BASE_DIRECT_JOINT_MAX_FORCE \
	BASE_DIRECT_ROOT_DAMPING \
	BASE_BRAKE \
	BASE_BRAKE_ENGAGE_LINEAR \
	BASE_BRAKE_ENGAGE_ANGULAR \
	BASE_BRAKE_CREEP_SPEED \
	BASE_BRAKE_CREEP_ANGULAR \
	BASE_BRAKE_K \
	BASE_BRAKE_C \
	BASE_BRAKE_MAX_FORCE \
	BASE_BRAKE_ANGULAR_K \
	BASE_BRAKE_ANGULAR_C \
	BASE_BRAKE_MAX_TORQUE \
	BASE_SETPOINT_MAX_LAG \
	BASE_SETPOINT_MAX_LAG_ANGULAR \
	BASE_GOAL_VELOCITY_TOLERANCE \
	BASE_JOINT_BRAKE \
	BASE_DIAG \
	CAMERA_FRAME_SKIP \
	RENDER_EVERY_N_STEPS \
	ENABLE_STEREO_CAMERAS \
	CAMERA_TUNE \
	RESET_DEEP_CAPTURE
SIM_ENV_ASSIGNMENTS = $(foreach v,$(SIM_ENV_NAMES),$(v)=$($(v)))
SIM_ENV_EXEC_FLAGS = $(foreach v,$(SIM_ENV_NAMES),-e $(v)=$($(v)))

help:
	@echo "Usage: make <action> [dev] [localhost] [compe_seed=1..4] [TIME=<秒>]"
	@echo ""
	@echo "Actions:"
	@echo "  build     Build images"
	@echo "  up        Start the stack (Isaac Sim + ROS2)"
	@echo "  down      Stop and remove containers"
	@echo "  logs      Tail logs"
	@echo "  ps        Container status"
	@echo "  exec      コンテナに入る: make exec ros2 / make exec isaacsim"
	@echo "  run       (dev) Run launch_isaacsim.py manually in isaacsim"
	@echo "  tune      観戦カメラの位置・画角を GUI で調整する (録画しない)"
	@echo "  tune-apply  tune のカメラ調整結果を configs/placement.yaml に反映する"
	@echo ""
	@echo "Modifiers (アクションと並べて書く):"
	@echo "  localhost  DDS を loopback に閉じる (例: make up localhost)"
	@echo "  dev        シミュレータを自動起動せずコンテナだけ立てる"
	@echo "  compe_seed  競技用の固定Seedを選ぶ (1〜4)"
	@echo "  nocache    レイヤキャッシュを使わず焼き直す (例: make build nocache)"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make build nocache # キャッシュ無しで全部焼き直す (数十分〜1時間以上)"
	@echo "  make up"
	@echo "  make up RVIZ=1    # RViz2 も起動する (既定はオフ)"
	@echo "  make up localhost # DDS を loopback に閉じる (LANへのマルチキャスト漏洩対策)"
	@echo "  make up localhost compe_seed=1   # 競技用の固定Seed 1で起動"
	@echo "  make up localhost ROS_DOMAIN_ID=31   # Apptainer 側と同じ値を指定する"
	@echo "  make up BASE_TRAJ_P_GAIN=1.0   # whole_body台車FBを弱める"
	@echo "  make up BASE_DIRECT_DRIVE=0    # 物理車輪駆動へ戻す"
	@echo "  make up BASE_KINEMATIC_DRIVE=0 # 位置追従をやめ平面ジョイントの動剛体へ戻す"
	@echo "                                 # (追従が崩れるので切り分け用。既定は 1)"
	@echo "  make up BASE_KINEMATIC_COLLISION=0 # 台車の壁クランプを外す (壁をすり抜ける)"
	@echo "  make up DRAWER_DAMPING=5       # 引き出しを軽くする (既定 15)"
	@echo "  make up DRAWER_PULL_LIMIT=0.20 # 引き出せる距離を短くする (既定 0.30m)"
	@echo "  make up DRAWER_JOINTS=0        # 引き出し化をやめ「置いてあるだけの箱」に戻す"
	@echo "  make up TIME=600   # 競技モード: 600秒(シミュ内時間)で自動終了。"
	@echo "                     # 4方向の観戦カメラで録画し recordings/ に mp4 保存"
	@echo "  make tune          # カメラ調整モード: GUI で 4台を動かすと"
	@echo "                     # recordings/tune/ に設定 yaml とプレビュー画像が出る"
	@echo ""
	@echo "  # Dev mode (開発用):"
	@echo "  make dev up     # start containers in background, isaacsim idle"
	@echo "  make dev run    # same terminal: run the sim, logs shown here"
	@echo "  make dev down   # stop when finished"

# Modifier "target" — accept as no-op so it can sit on the command line
dev:
	@:

# 同上。'make up localhost' の 'localhost' を no-op ターゲットとして受ける。
localhost:
	@:

# 同上。'make build nocache' の 'nocache' を no-op ターゲットとして受ける。
nocache:
	    @:

# 起動前に DDS プロファイルの指定を検証する。
# CYCLONEDDS_URI を手打ちして打ち間違えると、Cyclone は
# "can't open configuration file ..." を1行出すだけで先へ進み、その後
# rmw_create_node が失敗 -> Isaac 側は rclpy.node.Node() で例外 -> segfault
# という極めて分かりにくい落ち方をする。ここでコンテナを起動する前に弾く。
dds-check:
	@case '$(CYCLONEDDS_URI)' in \
	  file:///cyclonedds.xml|file:///cyclonedds.localhost.xml) ;; \
	  *) echo '❌ CYCLONEDDS_URI が不正です: $(CYCLONEDDS_URI)'; \
	     echo '   compose が bind mount しているのは次の2つだけです:'; \
	     echo '     file:///cyclonedds.xml            (既定/マルチキャスト版)'; \
	     echo '     file:///cyclonedds.localhost.xml  (make up localhost)'; \
	     echo '   手打ちせず "make up" / "make up localhost" を使ってください'; \
	     exit 1;; \
	esac
	@test -f 'assets/$(DDS_FILE)' || { \
	  echo '❌ assets/$(DDS_FILE) が見つかりません'; exit 1; }
	@echo '🔧 DDS: $(CYCLONEDDS_URI)   ROS_DOMAIN_ID=$(ROS_DOMAIN_ID)'
	@if [ -n '$(COMPE_SEED)' ]; then \
	  test -f 'configs/placement.compe_seed$(COMPE_SEED).yaml' || { \
	    echo '❌ 競技Seedファイルが見つかりません: configs/placement.compe_seed$(COMPE_SEED).yaml'; \
	    exit 1; \
	  }; \
	  echo '🎲 Competition Seed: $(COMPE_SEED)'; \
	fi

build:
	$(COMPOSE) build $(BUILD_FLAGS)

# TIME=<秒> を付けると競技モード (環境変数 TASK_TIME で isaacsim コンテナに渡る)。
up: dds-check
	TASK_TIME=$(TIME) GRASP_ATTACH=$(GRASP) USE_RVIZ=$(USE_RVIZ) $(SIM_ENV_ASSIGNMENTS) $(COMPOSE) up $(UP_FLAGS)

down:
	$(COMPOSE) down

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) ps -a

# --- exec: コンテナに入る ----------------------------------------------------
# 入れるコンテナは ros2 と isaacsim の 2 つ (cacheproxy は裏方なので入らない)。
#   make exec ros2       ROS 2 コンテナ
#   make exec isaacsim   Isaac Sim コンテナ
# 入り先は必ず書く (省略や単独指定はエラーにして使い方を出す)。
EXEC_SERVICE := $(filter ros2 isaacsim,$(MAKECMDGOALS))

# ros2 側は /ros_entrypoint.sh 経由で bash を起動する。これで ROS 環境
# (/opt/ros/humble + /ws の hsrb_interface 等) が source 済みの状態で入れる。
exec:
ifeq ($(EXEC_SERVICE),ros2)
	$(COMPOSE) exec ros2 /ros_entrypoint.sh bash
else ifeq ($(EXEC_SERVICE),isaacsim)
	$(COMPOSE) exec isaacsim bash
else
	@echo "Usage: make exec ros2       # ROS 2 コンテナ (ros2 topic list など)"
	@echo "       make exec isaacsim   # Isaac Sim コンテナ"
	@exit 1
endif

# exec の引数。必ず exec と一緒に使う (単独で打ったら使い方を出す)。
ros2 isaacsim:
ifeq ($(filter exec,$(MAKECMDGOALS)),)
	@echo "Usage: make exec $@"
	@exit 1
else
	@:
endif

run:
	$(COMPOSE) exec -e TASK_TIME=$(TIME) $(SIM_ENV_EXEC_FLAGS) isaacsim /ros_entrypoint.sh /isaac-sim/python.sh $(LAUNCH_PY)

# ============================================================
# カメラ調整モード
# ============================================================
# 観戦カメラ 4台だけ作って起動する (録画も自動終了もしない)。
# Isaac Sim の Viewport 左上のカメラ選択で Camera_north などを選ぶと、その
# カメラの映像がそのまま見える。動かすたびに次の 2 つが書き出される:
#   recordings/tune/arena_cameras.yaml  <- configs/placement.yaml に貼る設定
#   recordings/tune/preview.png         <- 実際に録画される 2x2 の絵
# (dev モードで使いたいときは `make dev up CAMERA_TUNE=1` + `make dev run`)
# 子 make には MAKECMDGOALS の 'localhost' が伝わらないので、選ばれたプロファイルを
# コマンドライン変数として明示的に引き継ぐ (make tune localhost を効かせるため)。
tune:
	@$(MAKE) up CAMERA_TUNE=1 CYCLONEDDS_URI=$(CYCLONEDDS_URI)

# tune の調整結果 (recordings/tune/arena_cameras.yaml) を
# configs/placement.yaml に反映する。make tune だけでは反映されない
# (コンテナから configs/ は読み取り専用のため)。
tune-apply:
	@python3 scripts/apply_tune.py
