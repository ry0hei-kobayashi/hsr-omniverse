# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

# from isaacsim.simulation_app import SimulationApp
#
# kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
# kit.set_setting("/app/extensions/installUntrustedExtensions", True)

# NOTE: Import ordering here is load-bearing. In an Isaac Sim standalone
# script, omni.*/isaacsim.* extension modules (omni.kit.commands, omni.isaac.*,
# omni.physx, isaacsim.core.*, pxr, ...) only become importable AFTER
# SimulationApp({...}) is instantiated -- that call boots Kit and puts the
# extension modules on sys.path. The only Isaac module safe to import before is
# the SimulationApp bootstrap itself. Do NOT let an auto-formatter (isort) hoist
# the post-SimulationApp imports above the kit = SimulationApp(...) call below;
# doing so reintroduces "ModuleNotFoundError: No module named 'omni.kit.commands'".

import os
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from isaacsim.simulation_app import SimulationApp

import object_placement  # no omni dependency; safe before Kit boots

kit = SimulationApp({
    'renderer': 'RayTracedLighting',
    'headless': False,
    'extra_args': [
        '--/app/extensions/excluded/0=isaacsim.asset.importer.urdf',
        '--/app/extensions/excluded/1=isaacsim.ros2.urdf',
    ],
})
kit.set_setting('/app/extensions/installUntrustedExtensions', True)

# isort: off
# --- The imports below require Kit to be running (SimulationApp booted above). ---
import omni.kit.commands
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.version import get_version
from isaacsim.sensors.physics import ContactSensor
from omni.isaac.core import SimulationContext
from omni.isaac.core.prims import GeometryPrim
from omni.isaac.core.utils import nucleus, stage, viewports
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.dynamic_control import _dynamic_control
from omni.physx import get_physx_simulation_interface
from omni.physx.scripts import utils as physx_utils
from pxr import (Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom,
                 UsdPhysics)
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty as EmptyMsg

# Local modules that import omni at module top -> must come after Kit boots.
import construct_environment
import arena_cameras
import furniture_spawn
import hsr
import people_spawn
# isort: on


# この実習リポジトリは ROS 2 (Humble) 専用 (元の hsr-omniverse にあった ROS1 対応は削除済み)。
import rclpy
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from std_srvs.srv import Empty as EmptySrv

# ============================================================
# 競技モード (TASK_TIME 環境変数)
# ============================================================
# `make up TIME=600` のように競技時間 (秒, シミュレータ内時間) を指定すると:
#   - 4方向の観戦カメラで録画 (arena_cameras.py)
#   - 競技時間が来たら動画を保存してシミュレータを自動終了
# 未指定 (0) なら従来どおり: カメラなし・時間無制限。
try:
    TASK_TIME = float(os.environ.get('TASK_TIME', '') or 0)
except ValueError:
    print(f"[task] WARNING: TASK_TIME={os.environ.get('TASK_TIME')!r} を数値として"
          f"読めません。競技モードは無効にします。")
    TASK_TIME = 0.0
if TASK_TIME > 0:
    print(f'[task] 競技モード: {TASK_TIME:.0f} 秒 (シミュレータ内時間) で自動終了・録画あり')

# ============================================================
# カメラ調整モード (CAMERA_TUNE 環境変数)
# ============================================================
# `make tune` で有効になる。観戦カメラ 4 台だけ作り、録画も自動終了もしない。
# GUI で位置・画角をいじると recordings/tune/ に設定 yaml とプレビュー画像が
# 書き出されるので、それを configs/placement.yaml に貼って確定させる。
CAMERA_TUNE = os.environ.get('CAMERA_TUNE', '').strip().lower() in (
    '1', 'true', 'yes', 'on')
if CAMERA_TUNE and TASK_TIME > 0:
    print('[task] CAMERA_TUNE=1 のため競技モード(録画)は無効にします。', flush=True)
    TASK_TIME = 0.0

# 物理 60 Hz のうち何ステップに 1 回描画するか。描画したフレームだけがカメラの
# publish 候補になるので、ここが RGB-D の上限レートを決める:
#   カメラ publish [Hz] = 60 / RENDER_EVERY_N_STEPS / (CAMERA_FRAME_SKIP + 1) * RTF
# 既定 2 = 30 Hz 描画。CAMERA_FRAME_SKIP=0 と合わせて実機同等の 30 Hz を狙う。
# (以前は 4 で、CAMERA_FRAME_SKIP=2 と合わせて 5 Hz 名目 = 実測 3.9 Hz だった。)
# RTF が 1.0 を保てないときは 3 や 4 に戻す。物理と全身制御は常に 60 Hz。
try:
    RENDER_EVERY_N_STEPS = max(
        1, int(os.environ.get('RENDER_EVERY_N_STEPS', '2')))
except ValueError:
    print(
        '[render] WARNING: RENDER_EVERY_N_STEPS=%r is invalid; using 2'
        % os.environ.get('RENDER_EVERY_N_STEPS'),
        flush=True,
    )
    RENDER_EVERY_N_STEPS = 2
print(
    '[render] viewport update every %d physics step(s)'
    % RENDER_EVERY_N_STEPS,
    flush=True,
)

# (初期視点と照明の配置は、部屋の中心が分かってからでないと決められないので
#  ワールドファイルを読んだ後 = 床サイズの計算の直後に行う)


# アセットの「根っこ(root)」。
# この実習構成では人 (people) や公式家具アセットを使わないため実質未使用だが、
# people_spawn / furniture_spawn の引数として渡すので既定の公開 URL を持っておく。
# 別サーバを使いたいときは環境変数 ISAAC_ASSETS_ROOT で上書きできる。
assets_root_path = os.environ.get(
    'ISAAC_ASSETS_ROOT',
    'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5',
)


# ============================================================
# 背景と床 (全タスク共通・オフライン対応)
# ============================================================
# 以前は Grid 環境 (/Isaac/Environments/Grid/default_environment.usd) をアセット
# サーバ(ネット)から読み込み、その GroundPlane を物理の床に使っていた。
#   (1) ネット無し(オフライン)でも起動できるようにする
#   (2) 背景を白い無地にする (青いグリッドをやめる)
# ため、ネット取得をやめてローカルに「白い背景(DomeLight) + 当たり判定付きの地面」を作る。
# 床の見た目テクスチャは後段の dressing がこの上に貼る。
BACKGROUND_STAGE_PATH = '/background'  # 互換のため名前だけ残す (contact 判定の文字列等)

# 白い背景: テクスチャ無しの DomeLight は、その色がそのまま背景として見える。
# 白すぎ/暗すぎる場合は inputs:intensity を調整する。
create_prim(
    '/World/WhiteBackground',
    'DomeLight',
    attributes={'inputs:intensity': 1000.0, 'inputs:color': (1.0, 1.0, 1.0)},
)

# ============================================================
# ワールドファイルの決定と床サイズの計算
# ============================================================
# ワールドは worlds/carrobo.world 固定 (実習用に 1 つだけ)。
# 床 (見た目のテクスチャ床と当たり判定の床) を部屋に合わせた大きさで作るため、
# 地面を作る前にワールドファイルを読んで部屋の外周を計算しておく。
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
world_candidates = [
    '/app/worlds/carrobo.world',  # コンテナ内 (イメージに ADD 済み / bind mount)
    os.path.join(repo_root, 'worlds', 'carrobo.world'),  # ホストで直接実行したとき
]
world_file = next((p for p in world_candidates if os.path.exists(p)), None)
if world_file is None:
    raise FileNotFoundError(
        f'worlds/carrobo.world が見つかりません (探した場所: {world_candidates})')
print(f'Loading world file: {world_file}')

# 床の一辺 = 部屋 (壁・家具の外周) + 余白。
# FLOOR_MARGIN を大きくすると、床が壁の外までその分だけ広がる。
# 見た目のテクスチャ床 (dressing) と当たり判定の床 (GroundPlane) を
# 同じ大きさ・同じ中心にそろえる (灰色の床がテクスチャの外にはみ出さない)。
FLOOR_MARGIN = 1.5  # 壁の外へ広げる余白 (m)
_wb = object_placement.world_xy_bounds(world_file)
if _wb is not None:
    _floor_size = max(_wb[1] - _wb[0], _wb[3] - _wb[2]) + 2.0 * FLOOR_MARGIN
    _floor_cx = (_wb[0] + _wb[1]) / 2.0
    _floor_cy = (_wb[2] + _wb[3]) / 2.0
else:
    _floor_size, _floor_cx, _floor_cy = 15.0, 0.0, 0.0
print(f'[floor] size={_floor_size:.2f}m center=({_floor_cx:.2f}, {_floor_cy:.2f})')

# ============================================================
# 初期視点と部屋の照明
# ============================================================
# どちらも「部屋の中心」を基準に置く。world 座標を直接書くと、ワールドを
# 平行移動・回転したときにここだけ取り残されて照明が偏る (実際に起きた)。
viewports.set_camera_view(
    eye=np.array([_floor_cx - 1.7, _floor_cy + 3.7, 5.0]),
    target=np.array([_floor_cx, _floor_cy, 0.0]))

# 部屋の中心をはさんで 2 灯。4m 離して対称に置く。
for _i, _dy in enumerate((2.0, -2.0)):
    create_prim(
        f'/World/Light_{_i + 1}',
        'SphereLight',
        position=np.array([_floor_cx, _floor_cy + _dy, 5.0]),
        attributes={'inputs:radius': 0.01, 'inputs:intensity': 5e4,
                    'inputs:color': (1.0, 1.0, 1.0)},
    )

# 物理の地面 (ロボット・物体が乗る面)。薄い板に当たり判定を付け、上面を z=0 に置く。
# 大きさは上で計算した「テクスチャ床と同じ」正方形。厚さは 5cm (横から見ても薄い板)。
_GROUND_THICKNESS = 0.05  # 床の厚さ (m)
_GROUND_PATH = '/World/GroundPlane'
_ground_prim = create_prim(
    prim_path=_GROUND_PATH,
    prim_type='Cube',
    # サイズ2の Cube なので scale = 実寸の半分。上面が z=0 に来るよう中心を厚さの半分だけ下げる。
    translation=[_floor_cx, _floor_cy, -_GROUND_THICKNESS / 2.0],
    scale=[_floor_size / 2.0, _floor_size / 2.0, _GROUND_THICKNESS / 2.0],
)
physx_utils.setCollider(_ground_prim, approximationShape='none')

# 床の摩擦 (滑り防止) をこのローカル地面に適用する。
floor_material = PhysicsMaterial(
    prim_path='/World/PhysicsMaterials/FloorMaterial',
    static_friction=60.0,
    dynamic_friction=60.0,
)
GeometryPrim(prim_path=_GROUND_PATH).apply_physics_material(
    floor_material, weaker_than_descendants=True
)

model_names = []
# 生オブジェクト(物理を持たない自作モデル)に drop_object で剛体を付けた prim パス。
# 剛体が spawn root に付き /body プリムにならないため、[recol] の /body 判定では
# 拾えない。ここに記録しておき、recol で同じ実行時メンテナンス(collider 再登録)を
# 適用してロボットがすり抜けないようにする。
_runtime_rigid_object_paths = []

# ============================================================
# reset_world
# ============================================================
_spawn_initial_states = []

# GUI (ギズモ) で動かされた prim を起動時の姿勢へ戻すためのテーブル。
# _spawn_initial_states は剛体を持つ物体しか持たないので、剛体を持たない静的 prim
# (world 由来の家具・壁, /World/Furniture, /World/People) は手で動かすと戻せない。
# それらは姿勢の真実が USD の xformOp だけにあるため、起動時 (timeline.play() の
# 直前) の op の値をここに控えてリセット時に書き戻す。
#
# 要素は prim 単位のエントリ:
#   {'prim': Usd.Prim, 'path': str,
#    'ops': [{'attr': Usd.Attribute, 'value': op の値}, ...],
#    'order_attr': Usd.Attribute, 'order': xformOpOrder の値 (未 authored なら None)}
#
# xformOpOrder も控えるのが要点。ギズモは op を持たない prim を掴むと
# xformOp:translate や xformOp:transform を新しく生やすので、op の値を書き戻す
# だけでは「後から生えた op」がスタックに残って prim が元に戻らない。
_manual_initial_xforms = []

# 上の記録対象にする「スポーン単位の root prim」パス。world 由来の家具・壁と
# drop_object の物体はどちらもステージ直下 (/<name>) に作られるが、そこには Kit の
# ビューポートカメラ (/OmniverseKit_Persp 等) も同居している。ステージ直下を丸ごと
# 走査するとリセットのたびにユーザの視点まで戻ってしまうので、作った物だけを控える。
_spawn_root_paths = []

# サービスコールバック (rclpy executor スレッド) から物理ステップ中に prim を
# 書き換えるのは危険なので、フラグを立ててメインループ側で物理ステップ間に実行する。
_reset_requested = False
_reset_done = threading.Event()


def _request_reset_and_wait():
    """reset_world を要求し、メインループが適用し終えるまでブロックする。"""
    global _reset_requested
    _reset_done.clear()
    _reset_requested = True
    # メインループが適用 → _reset_done.set() するまで待つ (適用保証)。
    _reset_done.wait(timeout=5.0)


def _manual_reset_target_prims():
    """GUI で掴んで動かしうる「スポーン単位の root prim」を集める。

    捕捉はこの root から下のサブツリー全体が対象 (_capture_manual_reset_targets)。
    ここで root だけを列挙するのは、ステージ直下を丸ごと走査すると Kit の
    ビューポートカメラ (/OmniverseKit_Persp 等) まで拾ってしまい、リセットのたびに
    ユーザの視点が戻ってしまうため。起点は「自分で作った物」に限る。
      - _spawn_root_paths: world 由来の家具・壁と drop_object の物体。
        ロボット (/hsrb) は reset_to_spawn が担当するので入っていない。
      - /World/Furniture, /World/People の子: furniture_spawn / people_spawn が置く。
        (/World 自身は床や環境テクスチャも含むので、この 2 つの子だけを見る)
    """
    _st = omni.usd.get_context().get_stage()
    prims = []
    for _path in _spawn_root_paths:
        _p = _st.GetPrimAtPath(_path)
        if _p and _p.IsValid():
            prims.append(_p)
    for _root_path in (furniture_spawn.FURNITURE_ROOT,
                       people_spawn.PEOPLE_ROOT):
        _root = _st.GetPrimAtPath(_root_path)
        if _root and _root.IsValid():
            prims.extend(_root.GetChildren())
    return prims


# サブツリーへ潜らず root の xformOp だけを控える旧挙動へ戻す非常口。
# 深い捕捉が重すぎて起動時間が問題になったとき用 (実測値は起動ログの
# [reset_world] captured ... に出る)。
_DEEP_CAPTURE = os.environ.get('RESET_DEEP_CAPTURE', '1') != '0'


def _capture_prim_xform_state(prim):
    """1 prim の xformOp とその順序を控える。Xformable でなければ None。"""
    _x = UsdGeom.Xformable(prim)
    if not _x:
        return None

    _ops = []
    for _op in _x.GetOrderedXformOps():
        _attr = _op.GetAttr()
        if _attr and _attr.IsValid():
            _ops.append({'attr': _attr, 'value': _attr.Get()})

    _order_attr = _x.GetXformOpOrderAttr()
    # authored されていなければ None として控える。復元時に順序を空にして、
    # ギズモが後から生やした op ごと無かったことにするため。
    _order = (_order_attr.Get()
              if _order_attr and _order_attr.HasAuthoredValue() else None)

    return {
        'prim': prim,
        'path': str(prim.GetPath()),
        'ops': _ops,
        'order_attr': _order_attr,
        'order': _order,
    }


def _capture_manual_reset_targets():
    """起動時の xformOp の値と順序を控える (reset_world で書き戻すため)。

    行列 1 個に畳まず op 単位で持つ。drop_object が Y-up モデルに後付けする
    rotateX(90) や furniture_spawn のスケール補正を含む op スタックの順序を
    壊さずに復元できる。

    スポーン root だけでなくその配下も走査する。tall table の天板のような
    「モデル内部のメッシュ」をギズモで掴まれると、root の op を戻しても
    子 prim に生えた op が残って戻らないため。走査は起動時の 1 回だけで、
    書き戻しは reset のときだけなので、実行中のフレームには乗らない。
    """
    _t0 = time.perf_counter()
    _prim_count = 0
    _op_count = 0

    for _root in _manual_reset_target_prims():
        try:
            for _p in (Usd.PrimRange(_root) if _DEEP_CAPTURE else (_root,)):
                if not _p.IsActive():
                    continue
                _state = _capture_prim_xform_state(_p)
                if _state is None:
                    continue
                _manual_initial_xforms.append(_state)
                _prim_count += 1
                _op_count += len(_state['ops'])
        except Exception as _e:
            print('[reset_world] xform capture failed for %s: %r'
                  % (_root.GetPath(), _e), flush=True)

    print('[reset_world] captured %d xform ops over %d prims for manual-move '
          'reset in %.0f ms (deep=%s)'
          % (_op_count, _prim_count, 1e3 * (time.perf_counter() - _t0),
             _DEEP_CAPTURE), flush=True)


def _restore_manual_xforms():
    """控えておいた xformOp を書き戻す (手で動かした prim を元の位置へ)。

    サブツリーまで控えているので対象は数千 prim になりうる。値が起動時と同じ prim
    には書き込まない。無駄な authoring と USD の変更通知を出さないためで、
    実際に動かされた prim は一部だけなので大半はここで素通りする。
    """
    for s in _manual_initial_xforms:
        for _op in s['ops']:
            _attr = _op['attr']
            if _op['value'] is None or not _attr.IsValid():
                # GUI で消された prim など。書き戻せないので飛ばす。
                continue
            try:
                if _attr.Get() != _op['value']:
                    _attr.Set(_op['value'])
            except Exception as _e:
                print('[reset_world] xform restore failed for %s: %r'
                      % (_attr.GetPath(), _e), flush=True)

        # 値を戻したあとに順序を戻す。ギズモが後から生やした op はここで
        # xformOpOrder から外れ、属性は残るが評価されなくなる。
        _prim = s['prim']
        _order_attr = s['order_attr']
        if not _prim.IsValid() or not _order_attr or not _order_attr.IsValid():
            continue
        try:
            if s['order'] is None:
                # 起動時は順序が未 authored = 変換なし。ギズモに掴まれて順序が
                # 生えた prim だけ空へ戻す (ClearXformOpOrder は空の順序を書くので、
                # 未 authored の prim に対して呼ぶと全 prim に無駄な opinion が残る)。
                if _order_attr.HasAuthoredValue():
                    UsdGeom.Xformable(_prim).ClearXformOpOrder()
            elif _order_attr.Get() != s['order']:
                _order_attr.Set(s['order'])
        except Exception as _e:
            print('[reset_world] xformOpOrder restore failed for %s: %r'
                  % (s['path'], _e), flush=True)


def _capture_spawn_rigid_state(root_prim, label):
    """スポーンした物の剛体のワールド姿勢を控える (reset_world で戻すため)。

    剛体 (RigidBodyAPI) が付いた prim を subtree から探し、その「合成済み
    ワールド変換」を保存する。ワールド変換で持つことで Y-up の rotateX(90) 補正や
    YCB の /body 内部オフセットが自動で正しく反映され、reset 時に
    dc.get_rigid_body(body_path) でそのまま戻せる。

    剛体 prim のパスはモデルによって違う (YCB と trofast は /<name>/body、
    wrc_* は /<name>/link) ので、パスを決め打ちせず subtree から探す。
    剛体を持たないモデル (壁の unit_box など) は何も控えない。
    """
    if not root_prim or not root_prim.IsValid():
        return
    try:
        _rb_prim = next(
            (p for p in Usd.PrimRange(root_prim)
             if p.HasAPI(UsdPhysics.RigidBodyAPI)),
            None,
        )
        if _rb_prim is None:
            return
        _m = UsdGeom.Xformable(_rb_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        _t = _m.ExtractTranslation()
        _q = _m.GetOrthonormalized().ExtractRotationQuat()
        _qi = _q.GetImaginary()
        _spawn_initial_states.append({
            'body_path': str(_rb_prim.GetPath()),
            'p': (_t[0], _t[1], _t[2]),
            'q': (_q.GetReal(), _qi[0], _qi[1], _qi[2]),  # (w, x, y, z)
        })
    except Exception as _e:
        print('[reset_world] spawn pose capture failed for %s: %r' %
              (label, _e), flush=True)


model_root = os.path.join(repo_root, 'usd', 'wrs_models')
if not os.path.exists(model_root):
    model_root = '/app/usd/wrs_models'

# 競技当日に追加する未知物体。wrs_models と同じく
#   <object名>/model.usd
# の構成にしておけば placement.yaml の object: から参照できる。
unknown_object_root = os.path.join(repo_root, 'usd', 'unknown_objects')
if not os.path.exists(unknown_object_root):
    unknown_object_root = '/app/usd/unknown_objects'

# YCB 以外の「物理設定を持たないモデル」を読み込んだときに付ける既定の質量 (kg)。
# YCB は 1 つずつ実測値が model.usd に入っているので、この値は通常使われない。
DEFAULT_OBJECT_MASS_KG = 0.2

# ============================================================
# ワールド (部屋) の読み込み
# ============================================================
# ワールドファイル (world_file) は地面を作る前に上で決定済み。
# ここでは家具・壁の配置を読み取って部屋を組み立てる。
tree = ET.parse(world_file)
root = tree.getroot()
for i in root.findall('world/include'):
    model_name = i.find('name').text
    model_uri = i.find('uri').text
    (x, y, z, er, ep, ey) = [float(n) for n in i.find('pose').text.split(' ')]
    stage_path = f'/{model_name}'
    model_path = os.path.join(
        model_root, model_uri.replace('model://', ''), 'model.usd')
    if model_uri == 'model://unit_box' and not os.path.exists(model_path):
        scale_tag = i.find('scale')
        if scale_tag is None:
            size = np.array([1.0, 1.0, 1.0])
        else:
            size = np.array([float(n) for n in scale_tag.text.split(' ')])
        size = np.maximum(size, 1e-4)
        orientation = euler_angles_to_quat([er, ep, ey])
        create_prim(
            prim_path=stage_path,
            prim_type='Cube',
            translation=[x, y, z],
            orientation=orientation,
            scale=size * 0.5,
        )
        # 家具・壁の Cube に当たり判定 (collider) を付ける。
        # RigidBodyAPI は付けないので「動かない固い箱」になり、この上に置いた
        # 物体 (YCB 等) が天板に乗って止まる (collider が無いとすり抜けて落ちる)。
        _cube_prim = omni.usd.get_context().get_stage().GetPrimAtPath(stage_path)
        physx_utils.setCollider(_cube_prim, approximationShape='none')
        model_names.append(model_name)
        _spawn_root_paths.append(stage_path)
    if model_uri == 'model://unit_cylinder' and not os.path.exists(model_path):
        # unit_box と同様に、USD モデルが無い円柱は Cylinder prim で直接作る。
        # scale は (直径, 直径, 高さ) 指定 → prim には半分を渡す (Cube と同じ扱い)。
        scale_tag = i.find('scale')
        if scale_tag is None:
            size = np.array([1.0, 1.0, 1.0])
        else:
            size = np.array([float(n) for n in scale_tag.text.split(' ')])
        size = np.maximum(size, 1e-4)
        orientation = euler_angles_to_quat([er, ep, ey])
        create_prim(
            prim_path=stage_path,
            prim_type='Cylinder',
            translation=[x, y, z],
            orientation=orientation,
            scale=size * 0.5,
        )
        # 当たり判定 (collider)。円柱は 'none' (三角メッシュ) が使えないことが
        # あるため convexHull で近似する (テーブルとしては十分な精度)。
        _cyl_prim = omni.usd.get_context().get_stage().GetPrimAtPath(stage_path)
        physx_utils.setCollider(_cyl_prim, approximationShape='convexHull')
        model_names.append(model_name)
        _spawn_root_paths.append(stage_path)
    if not os.path.exists(model_path):
        continue
    create_prim(
        prim_path=stage_path,
        prim_type='Xform',
        translation=[x, y, z],
        orientation=euler_angles_to_quat([er, ep, ey]),
    )
    stage.add_reference_to_stage(model_path, Sdf.Path(stage_path))
    if i.find('static') is not None:
        # Create fixed joint between the world if the object is static
        root_joint = UsdPhysics.FixedJoint.Define(
            omni.usd.get_context().get_stage(), stage_path + '/root_joint'
        )
        root_joint.CreateBody1Rel().SetTargets([stage_path + '/link'])
        root_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
        root_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        root_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        root_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    else:
        # reset_world 用: <static> の無いモデルは FixedJoint で固定されず、
        # ロボットに押されると動く動的剛体 (trofast_*, wrc_tray_*, wrc_container_*)。
        # 参照した model.usd 側に RigidBodyAPI が入っているので、剛体のワールド姿勢を
        # 控えて _reset_objects() が dynamic control で戻せるようにする。親 Xform の
        # xformOp を書き戻すだけでは PhysX が持つ姿勢を上書きできない。
        # <static> 付きは FixedJoint があるので対象にしない (dc で姿勢を書くと
        # ジョイントと競合する)。まだ play() していないので、ここで控えるのは
        # world ファイルどおりの姿勢になる。
        _capture_spawn_rigid_state(
            omni.usd.get_context().get_stage().GetPrimAtPath(stage_path),
            model_name)
    model_names.append(model_name)
    _spawn_root_paths.append(stage_path)


# ============================================================
# 引き出し (Room SW の trofast) を「本物の引き出し」にする
# ============================================================
# world ファイルの trofast_* は段違い棚 (wrc_stair_like_drawer) に「置いてあるだけ」の
# 自由剛体で、そのままではロボットが取っ手を掴んで引くことができない。理由は 2 つ:
#
#   1. trofast_knob/model.usd の /body に ArticulationRootAPI が付いている。
#      これが付いた物体は PhysX が別アーティキュレーションとして扱い、ロボット
#      (これもアーティキュレーション) と衝突しなくなる = 指がすり抜ける。
#      YCB は drop_object() の [obj-fix] で除去しているが、world の <include> 経由で
#      読まれる trofast はその経路を通らないため付いたままだった。
#
#   2. 棚のレール間隔は 0.280 m なのに箱の上縁は 0.300 m あり、左右 10 mm ずつ
#      最初から食い込んでいる。実物の「縁をレールに引っ掛けて吊る」構造をそのまま
#      USD にしたためで、剛体シミュレーションでは初期めり込みになる。PhysX が
#      これを押し出そうとして箱が弾かれたり楔状に噛んで沈み込んだりする。
#
# 対策として、棚と箱の間に PrismaticJoint (直動ジョイント) を張り、レールの代わりに
# ジョイントで運動を拘束する。ジョイントで繋いだ 2 体は collisionEnabled=False により
# 当たり判定が切れるので、上記 2. のめり込みも同時に解消する。結果として
# 「手前にだけスライドし、引き切ると止まり、抜け落ちない」本物の引き出しになる。
DRAWER_FRAME_PATH = '/wrc_stair_like_drawer'
# 引き出しを何 m 引き出せるか (ジョイントの上限)。箱の奥行きは 0.4235 m。
DRAWER_PULL_LIMIT = float(os.environ.get('DRAWER_PULL_LIMIT', '0.30'))
# 引き出しの粘性抵抗 [N/(m/s)]。大きいほど重い引き心地になる。0 で無抵抗。
# 指の把持力 (hsr.py の FINGER_MAX_FORCE=10) で引ける範囲に収めること。
DRAWER_DAMPING = float(os.environ.get('DRAWER_DAMPING', '15.0'))
# 粘性抵抗が出せる力の上限 [N]。引き出しを止める力ではなく減衰の頭打ち。
DRAWER_DAMPING_MAX_FORCE = float(os.environ.get('DRAWER_DAMPING_MAX_FORCE', '200.0'))
# 0 にすると引き出し化を丸ごと止めて従来の「置いてあるだけの箱」に戻せる。
DRAWER_JOINTS_ENABLED = os.environ.get('DRAWER_JOINTS', '1') != '0'

# 引き出しにした箱の /body パス。[recol] の対象外にするために覚えておく
# ([recol] は setRigidBody で剛体を作り直すため、張ったジョイントが壊れる)。
_drawer_body_paths = []


def _strip_articulation_root(prim):
    """prim 以下の ArticulationRootAPI を外して「ただの剛体」に戻す。

    drop_object() の [obj-fix] と同じ処理。付いたままだとロボットの指がすり抜ける。
    """
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            print('[drawer] removed ArticulationRootAPI from %s' % p.GetPath(),
                  flush=True)
        if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
            p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)


def _setup_drawers():
    """段違い棚と trofast_* を PrismaticJoint で繋いで引き出しにする。"""
    _stage = omni.usd.get_context().get_stage()
    frame = _stage.GetPrimAtPath(DRAWER_FRAME_PATH)
    frame_link = _stage.GetPrimAtPath(DRAWER_FRAME_PATH + '/link')
    if not frame.IsValid() or not frame_link.IsValid():
        # この world に段違い棚が無い (別アリーナ) なら何もしない。
        return

    # 棚もアーティキュレーションを外して素の剛体に戻したうえで kinematic にする。
    # <static> により world への FixedJoint は張られているが、それだけだと剛体
    # (mass 1kg) のままなので引き出しを引く反力で棚が揺れる。kinematic にすれば
    # 完全に不動のアンカーになり、ジョイントの相手として安定する。
    _strip_articulation_root(frame)
    UsdPhysics.RigidBodyAPI.Apply(frame_link).CreateKinematicEnabledAttr(True)

    frame_inv = UsdGeom.Xformable(frame_link).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).GetInverse()

    for name in model_names:
        if not name.startswith('trofast'):
            continue
        root = _stage.GetPrimAtPath('/' + name)
        body_path = '/' + name + '/body'
        body = _stage.GetPrimAtPath(body_path)
        if not root.IsValid() or not body.IsValid():
            print('[drawer] skip %s (no /body prim)' % name, flush=True)
            continue

        # 1. 指がすり抜ける原因の ArticulationRootAPI を外す。
        _strip_articulation_root(root)

        # 2. 棚に対する箱の現在の相対姿勢を測り、そこをジョイント原点 (= 閉じた状態)
        #    にする。world ファイルに書かれた姿勢がそのまま「閉」になるので、
        #    座標をコードに焼き込まずに済む。
        body_l2w = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        rel = body_l2w * frame_inv
        rel_pos = rel.ExtractTranslation()
        rel_rot = rel.GetOrthonormalized().ExtractRotationQuat()

        # 3. 直動ジョイント。軸は棚ローカルの X = ワールド +Y = 引き出しの手前方向。
        #    (棚は yaw=90° で置かれ、前面の桟が world の +Y 側にある)
        joint = UsdPhysics.PrismaticJoint.Define(
            _stage, Sdf.Path('/' + name + '/drawer_joint'))
        joint.CreateBody0Rel().SetTargets([DRAWER_FRAME_PATH + '/link'])
        joint.CreateBody1Rel().SetTargets([body_path])
        joint.CreateAxisAttr('X')
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel_pos))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(rel_rot))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        # 0 = 閉じきり、DRAWER_PULL_LIMIT = 引ききり。ここで機械的に止まる。
        joint.CreateLowerLimitAttr(0.0)
        joint.CreateUpperLimitAttr(DRAWER_PULL_LIMIT)
        # 棚と箱の当たり判定を切る (既定でも False だが、レールとの初期めり込みを
        # 確実に無効化したいので明示する)。動きはジョイントが拘束するので、
        # 当たり判定を切っても箱が棚をすり抜けて落ちることはない。
        joint.CreateCollisionEnabledAttr(False)

        # 4. 粘性抵抗。これが無いと引いた勢いでストッパーに激突して跳ね返る。
        #    stiffness=0 なので「戻ろうとする力」は働かず、引いた位置で止まる。
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(DRAWER_DAMPING)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateMaxForceAttr(DRAWER_DAMPING_MAX_FORCE)

        # 5. 掴んで静止すると眠ってしまい、次に押しても反応しなくなるのを防ぐ
        #    ([recol] が同じことをしているが、引き出しは [recol] 対象外にするため)。
        _rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
        _rb.CreateDisableGravityAttr(False)
        _rb.CreateSleepThresholdAttr(0.0)

        _drawer_body_paths.append(body_path)
        print('[drawer] %s -> prismatic joint (0..%.2f m, damping=%.1f)'
              % (name, DRAWER_PULL_LIMIT, DRAWER_DAMPING), flush=True)


if DRAWER_JOINTS_ENABLED:
    try:
        _setup_drawers()
    except Exception as _e:
        # 引き出しが作れなくてもシミュレータ自体は起動させる (実習を止めない)。
        print('[drawer] setup failed: %r' % _e, flush=True)
else:
    print('[drawer] DRAWER_JOINTS=0 のため引き出し化をスキップ', flush=True)


def _subtree_has_rigid_body(prim):
    """prim とその子孫のどこかに既に剛体 (RigidBodyAPI) が付いているか調べる。

    YCB など model.usd 自身に物理が入っているモデルは True を返す。
    物理設定を持たないモデルは False。
    """
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return True
    return False


def _ensure_default_mass(prim, mass_kg=DEFAULT_OBJECT_MASS_KG):
    """剛体 prim に質量がまだ無ければ既定値を入れる。

    setRigidBody だけだと質量が未指定で、PhysX が「密度 × 当たり判定の体積」から
    自動推定する。これはモデルの大きさで大きくブレるため、自作モデルには一律の
    既定質量 (DEFAULT_OBJECT_MASS_KG) を入れて把持挙動を安定させる。
    既に質量が書かれていれば (YCB など) 触らない。
    """
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_attr = mass_api.GetMassAttr()
    if not mass_attr or not mass_attr.HasAuthoredValue() or not mass_attr.Get():
        mass_api.CreateMassAttr(float(mass_kg))


def drop_object(gazebo_name, name, x, y, z, yaw=0.0, roll=0.0, pitch=0.0):
    global model_names
    # USD の prim パスは "/" が階層区切りになるため、name に相対パス
    # (rc26_practice_day_1/drink/led 等) が含まれると壊れる。"-" と "/" を
    # まとめて "_" に置換し、1 階層の安全な prim 名にする。
    safe_name = gazebo_name.replace('-', '_').replace('/', '_')
    stage_path = f'/{safe_name}'
    # name は usd/wrs_models/ または usd/unknown_objects/ 内のフォルダ名。
    # 例: 'ycb_011_banana', 'logitech_m310_mouse'
    model_candidates = [
        os.path.join(model_root, name, 'model.usd'),
        os.path.join(unknown_object_root, name, 'model.usd'),
        '/app/usd/wrs_models/' + name + '/model.usd',
        '/app/usd/unknown_objects/' + name + '/model.usd',
    ]
    model_path = next((p for p in model_candidates if os.path.exists(p)), None)
    if model_path is None:
        print(f'Model not found for {name}: tried {model_candidates}')
        return None

    create_prim(
        prim_path=stage_path,
        prim_type='Xform',
        translation=[x, y, z],
        orientation=euler_angles_to_quat([roll, pitch, yaw]),
    )
    stage.add_reference_to_stage(model_path, Sdf.Path(stage_path))

    # Object Capture (iPhone 撮影) 製などは upAxis=Y で作られており、Z-up の
    # 世界ではそのままだと横倒しになる。参照の「後」に rotateX(90) を足して
    # 立たせる。create_prim が [translate, orient] を設定済みなので、ここで足すと
    # 順序が [translate, orient, rotateX] になり、ジオメトリにはまず rotateX
    # (Y-up→Z-up) → 次に向き → 最後に位置、の順で効く。
    # ※ モデルのルートに焼き込んだ回転は create_prim の設定に上書きされて効かない
    #   ため、必ずこのコード側で足す (people_spawn.py が人を立たせるのと同じ手法)。
    # ※ Z-up の YCB / wrs_models には足さない (upAxis を見て判定)。
    try:
        _src_stage = Usd.Stage.Open(model_path)
        _is_y_up = UsdGeom.GetStageUpAxis(_src_stage) == UsdGeom.Tokens.y
    except Exception:
        _is_y_up = False
    if _is_y_up:
        _prim = omni.usd.get_context().get_stage().GetPrimAtPath(stage_path)
        UsdGeom.Xformable(_prim).AddRotateXOp().Set(90.0)

    # 物理 (衝突判定 + 重力) を保証する。
    # YCB などは model.usd 自身に剛体が入っているのでそのまま使う。
    # led のように物理を持たない自作モデルには、ここで剛体 + 当たり判定を付けて、
    # 他の物体と同じように天板へ落ちて乗るようにする (浮いたままにならない)。
    dropped_prim = omni.usd.get_context().get_stage().GetPrimAtPath(stage_path)
    if not _subtree_has_rigid_body(dropped_prim):
        # convexHull = 物体の外形を凸形状で近似した当たり判定。
        # (動く物体の標準。三角メッシュ 'none' は静止物専用で落下に使えない)
        # これで YCB と同じ「剛体 + convexHull の当たり判定」が読み込み時に
        # 自動で付くので、自作モデルの model.usd を手作業で編集しなくてよい。
        physx_utils.setRigidBody(dropped_prim, 'convexHull', False)
        # 質量も既定値を入れて YCB 相当の構成にする (上のコメント参照)。
        _ensure_default_mass(dropped_prim)
        # 剛体は spawn root に付き /body にはならないので、[recol] が拾えるように記録する。
        _runtime_rigid_object_paths.append(stage_path)

    # 物体に ArticulationRootAPI が付いていると PhysX が「アーティキュレーション」として
    # 扱い、ロボット(別アーティキュレーション)と衝突しなくなる(静的な机/床とは衝突するが、
    # 腕が全部すり抜ける)。YCB の model.usd にこれが入っているため、ここで除去して
    # 物体を単なる剛体に戻す。これでロボットの指/腕が物体に当たるようになる。
    try:
        from pxr import PhysxSchema as _PXS

        for _op in Usd.PrimRange(dropped_prim):
            if _op.HasAPI(UsdPhysics.ArticulationRootAPI):
                _op.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                print('[obj-fix] removed ArticulationRootAPI from %s' %
                      _op.GetPath(), flush=True)
            if _op.HasAPI(_PXS.PhysxArticulationAPI):
                _op.RemoveAPI(_PXS.PhysxArticulationAPI)
    except Exception as _e:
        print('[obj-fix] err %r' % _e, flush=True)

    model_names.append(gazebo_name)
    _spawn_root_paths.append(stage_path)

    # reset_world 用: この物体の初期姿勢を記録する。
    _capture_spawn_rigid_state(dropped_prim, gazebo_name)

    return model_path


# configs/placement.yaml の指定に従って家具の上に物体 (YCB) を配置する。
object_placement.apply_placements(world_file, drop_object)

# placement.yaml の people: セクションに従って「人」を配置する。
# 既定では people.list が空なので何も置かれない (呼び出しは常に残す)。
# 人モデル/モーションは usd/isaac_offline/ に同梱済みなので、placement.yaml に
# 人を書けばネット無しでそのまま出せる。
# 戻り値: (配置人数, ループ開始秒, ループ終了秒)。
_num_people, _people_loop_start, _people_loop_end = people_spawn.spawn_people(
    assets_root_path, kit
)

# placement.yaml の furniture: セクションに従って「本物のメッシュの家具(机/椅子)」を
# 配置する。この実習構成では furniture: を書いていないので何も置かない no-op。
furniture_spawn.spawn_furniture(assets_root_path, kit)

# HSR の初期スポーン位置 (map 座標 = world 座標) は configs/placement.yaml の
# robot: セクションで定義する (robot/objects/people をまとめた設定ファイル)。
# 注意: env_furniture の operator_position は「人」の位置であって、ロボットの位置ではない。
_robot_spawn = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}  # 設定ファイルが無いときのフォールバック
_spawn_candidates = [
    '/app/configs/placement.yaml',
    os.path.join(repo_root, 'configs', 'placement.yaml'),
]
_spawn_path = next((p for p in _spawn_candidates if os.path.exists(p)), None)
if _spawn_path is not None:
    with open(_spawn_path) as _f:
        _cfg = yaml.safe_load(_f) or {}
    _robot_cfg = _cfg.get('robot') or {}  # robot: セクション (位置 x/y/yaw のみ使う)
    for _k in ('x', 'y', 'yaw'):
        if _robot_cfg.get(_k) is not None:
            try:
                _robot_spawn[_k] = float(_robot_cfg[_k])
            except (TypeError, ValueError):
                # 数値でない (例: 小数点を ',' で書いた) 場合でも sim を落とさず継続。
                print(
                    f'[hsr] WARNING: placement.yaml の robot.{_k}={_robot_cfg[_k]!r} は'
                    f'数値として読めません。フォールバック {_robot_spawn[_k]} を使用 '
                    f"(小数点は '.' で書いてください)。"
                )
    print(f'[hsr] spawn from {_spawn_path} (robot:): {_robot_spawn}')
else:
    print(f'[hsr] placement.yaml が無いのでフォールバック値を使用: {_robot_spawn}')

# スポーン姿勢は odom (ひいては map) 座標の原点そのもの (scripts/hsr.py:3243)。
# 原点からずれていると、placement.yaml に書く world 座標と、ナビゲーションに
# 渡す map 座標が、そのずれの分だけ食い違う。黙って壊れると原因が分からない
# ので、はっきり警告する。位置を変えたいときは部屋 (worlds/carrobo.world) の
# ほうを動かして、スタート地点が原点に来るようにすること。
if max(abs(_robot_spawn['x']), abs(_robot_spawn['y']),
       abs(_robot_spawn['yaw'])) > 1e-6:
    print(
        '[hsr] WARNING: robot: が原点 (0, 0, 0) ではありません '
        f"(x={_robot_spawn['x']}, y={_robot_spawn['y']}, "
        f"yaw={_robot_spawn['yaw']}).\n"
        '[hsr]          odom / map 座標が world 座標とこの分ずれます。'
        '絶対座標のナビゲーションを使うなら\n'
        '[hsr]          robot: を (0, 0, 0) に戻し、代わりに '
        'worlds/carrobo.world 側を平行移動してください。',
        flush=True,
    )

# ロボットは HSR-B (hsrb) 固定。
hsr_stage_path = '/hsrb'
create_prim(
    prim_path=hsr_stage_path,
    prim_type='Xform',
    translation=[_robot_spawn['x'], _robot_spawn['y'], 0],
    orientation=euler_angles_to_quat([0, 0, _robot_spawn['yaw']]),
)

_hsr = hsr.hsr(stage_path=hsr_stage_path)
model_names.append('hsrb')

import std_msgs.msg

collision_detect_pub = _hsr.ros2node.create_publisher(
    std_msgs.msg.Bool,
    '/undesired_contact_detector/detect',
    qos_profile=rclpy.qos.qos_profile_system_default,
)

contact_links = [
    '/hsrb/hsrb/base_link/collisions',
    '/hsrb/hsrb/base_f_bumper_link/collisions',
    '/hsrb/hsrb/base_b_bumper_link/collisions',
]

contact_sensors = []
stage_handle = omni.usd.get_context().get_stage()
for i in range(len(contact_links)):
    contact_report_api = PhysxSchema.PhysxContactReportAPI.Apply(
        stage_handle.GetPrimAtPath(contact_links[i])
    )
    contact_report_api.CreateThresholdAttr(0.0)
    contact_sensors.append(
        ContactSensor(
            prim_path=f'{contact_links[i]}/Contact_Sensor',
            name='Contact_Sensor',
            frequency=10,
            min_threshold=0,
            radius=-1,
        )
    )


actor_to_body_path_cache = {}
prev_contact = None


def _actor_body_path(actor):
    try:
        return actor_to_body_path_cache[actor]
    except KeyError:
        path = str(PhysicsSchemaTools.intToSdfPath(actor))
        actor_to_body_path_cache[actor] = path
        return path


def contact_report_event(ch, cd):
    global prev_contact
    for c in ch:
        path0 = _actor_body_path(c.actor0)
        path1 = _actor_body_path(c.actor1)
        robot0 = path0 == '/hsrb' or path0.startswith('/hsrb/')
        robot1 = path1 == '/hsrb' or path1.startswith('/hsrb/')

        # PhysX の購読はシーン全体の接触を返す。従来は actor1 だけを見て
        # wall と World (床) の常時接触までロボットの衝突として通知していた。
        # ロボットが当事者でない接触と、ロボット内部の自己接触は無視する。
        if robot0 == robot1:
            continue
        other_path = path1 if robot0 else path0
        body_name = other_path.strip('/').split('/')[0]
        if body_name != 'background' and prev_contact != body_name:
            print(f'Contact {body_name}')
            prev_contact = body_name
        # 壁 (wall_*) にロボット自身がぶつかった場合だけ衝突検出トピックへ
        # 知らせる (競技の Hit 判定と同じ仕組み)。
        if body_name.startswith('wall_'):
            collision_detect_pub.publish(std_msgs.msg.Bool(data=True))


# this variable is unused, but it is required to continue the subscription
_contact_report_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(
    contact_report_event
)

# Start simulation
kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
# [usdsync] 物理結果を USD にも毎フレーム書き戻す設定。
#   既定(False)では物理は Fabric(描画用の速いメモリ)だけに書かれ、USD はスポーン時の値で
#   凍結する→ギズモ/Property の数値だけが置いてけぼりになり「見た目と数値がズレる」。
#   True にすると物理位置が USD にも反映され、見た目・当たり判定・物理・ギズモが常に同じ位置に
#   そろう(数値も生の物理に追従)。代償は毎フレームの USD 書き込みでわずかに描画が重くなること。
kit.set_setting('/physics/updateToUsd', True)
kit.update()
_hsr.onsimulationstart(simulation_context)
simulation_context.initialize_physics()
# reset_world 用: 物理が 1 ステップも進んでいない今 (= placement.yaml / world ファイル
# どおりの姿勢) の xformOp を控える。この後に GUI で prim を動かしても、リセットで
# ここに戻せるようになる。play() より後だと物体が落ち始めた姿勢を拾ってしまう。
_capture_manual_reset_targets()
omni.timeline.get_timeline_interface().play()

# 人を配置したときだけアニメーションをループ再生する設定にする。
# (人が居ないときはタイムラインに触れず、従来どおりの挙動を保つ。)
# set_looping だけだと「どこで折り返すか」が分からずアニメが最後のポーズで
# 止まってしまう。ループ周期 (_people_loop_duration) を end_time に設定して
# はじめてループする (復元ガイドの知見)。
# この周期は people_spawn 側で「一番短いクリップ」に決めている。タイムラインは
# シーンに 1 本だけで全員が共有するため、こうしないと短いクリップの人が
# 長いクリップの人を待つ間フリーズしてしまうため (詳細は people_spawn.py)。
if _num_people > 0:
    _timeline = omni.timeline.get_timeline_interface()
    # 再生区間を [開始秒, 終了秒] に絞ってループさせる。
    # loop_window で「手を上げて振っている区間」だけを指定すると、手を下ろす
    # 部分が再生範囲から外れ、上げっぱなしで振り続けているように見える。
    if _people_loop_end > 0.0:
        _timeline.set_start_time(_people_loop_start)
        _timeline.set_end_time(_people_loop_end)
        # 再生ヘッドを区間の先頭に置いてから始める (区間外から始まらないように)。
        _timeline.set_current_time(_people_loop_start)
    _timeline.set_looping(True)
    print(
        f'[people] timeline looping on for {_num_people} character(s), '
        f'window={_people_loop_start:.2f}s..{_people_loop_end:.2f}s'
    )

# ラボ環境テクスチャ (床 + 周囲背景 + 照明) を適用。
# timeline.play() の "後" でないと PhysX セットアップを壊すので注意。
# 床のサイズ・中心は起動時に計算した _floor_size/_floor_cx/_floor_cy を使う
# (当たり判定の GroundPlane と同じ大きさ・中心 = 灰色の床がはみ出さない)。
# preset/lighting は configs/dressing.yaml の defaults がそのまま使われる。
print(
    f'[dressing] room_size={_floor_size:.2f} '
    f'center=({_floor_cx:.2f}, {_floor_cy:.2f})'
)
construct_environment.apply_lab_dressing(
    room_size=_floor_size,
    center_x=_floor_cx,
    center_y=_floor_cy,
)
for _ in range(3):
    kit.update()

# 競技モードなら 4方向の観戦カメラを作って録画を開始する。
# (timeline.play() と dressing の後 = シーンが完成した状態で作る)
_recorder = None
if TASK_TIME > 0:
    _recorder = arena_cameras.ArenaRecorder(
        TASK_TIME, center_x=_floor_cx, center_y=_floor_cy)
    _recorder.setup()

# 調整モードなら同じカメラを作るだけ (録画なし・時間制限なし)。
_tuner = None
if CAMERA_TUNE:
    _tuner = arena_cameras.ArenaCameraTuner(
        center_x=_floor_cx, center_y=_floor_cy)
    _tuner.setup()


# simulate gazebo ros APIs required for task evaluators
def get_xform(stage, model_name):
    try:
        name = model_name.replace('::link', '').replace('-', '_')
        prim = stage.GetPrimAtPath(f'/{name}/link')
        if not prim.IsValid():
            prim = stage.GetPrimAtPath(f'/{name}/body')
        if not prim.IsValid() and f'/{name}' in _runtime_rigid_object_paths:
            # 生オブジェクト: 剛体は /body ではなく spawn root に付く。
            prim = stage.GetPrimAtPath(f'/{name}')
        if not prim.IsValid():
            prim = stage.GetPrimAtPath(f'/{name}/hsrb/base_footprint')
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    except:
        # print(f'Failed to get xform for {model_name}')
        return Gf.Matrix4d()


def handle_get_world_properties_ros2(req, ret):
    ret.model_names = model_names
    ret.success = True
    return ret


def handle_get_model_state_ros2(req, ret):
    stage = omni.usd.get_context().get_stage()
    objxform = get_xform(stage, req.model_name)
    refxform = get_xform(stage, req.relative_entity_name)
    relpose = objxform * refxform.GetInverse()
    translation = relpose.ExtractTranslation()
    rotation = relpose.GetOrthonormalized().ExtractRotationQuat()
    rotation_imaginary = rotation.GetImaginary()
    # create response
    ret.header.frame_id = req.relative_entity_name
    ret.pose.position.x = translation[0]
    ret.pose.position.y = translation[1]
    ret.pose.position.z = translation[2]
    ret.pose.orientation.x = rotation_imaginary[0]
    ret.pose.orientation.y = rotation_imaginary[1]
    ret.pose.orientation.z = rotation_imaginary[2]
    ret.pose.orientation.w = rotation.GetReal()
    ret.success = True
    return ret


_hsr.ros2node.create_service(
    GetWorldProperties,
    '/gazebo/get_world_properties',
    handle_get_world_properties_ros2,
    qos_profile=rclpy.qos.qos_profile_services_default,
)
_hsr.ros2node.create_service(
    GetModelState,
    '/gazebo/get_model_state',
    handle_get_model_state_ros2,
    qos_profile=rclpy.qos.qos_profile_services_default,
)


def handle_reset_world_ros2(req, ret):
    _request_reset_and_wait()
    return ret


_hsr.ros2node.create_service(
    EmptySrv,
    '/isaac/reset_world',
    handle_reset_world_ros2,
    qos_profile=rclpy.qos.qos_profile_services_default,
)


# トピック版リセット。`ros2 topic pub --once /isaac/reset_world_request
# std_msgs/msg/Empty` の 1 コマンドでリセットできる (サービスと同じ経路)。
# コールバックはフラグを立てるだけ。実際の復元はメインループが物理ステップ間で行う。
def handle_reset_world_topic_ros2(_msg):
    global _reset_requested
    _reset_requested = True


_hsr.ros2node.create_subscription(
    EmptyMsg,
    '/isaac/reset_world_request',
    handle_reset_world_topic_ros2,
    10,
)

# reset_world (テレポート) 後に localization スタックを spawn 位置へ復帰させる publisher。
#   /isaac/reset_world_event : laser_scan_matcher 再起動ヘルパー (別コンテナの ROS ノード)
#                              への通知。matcher はテレポートで参照 scan が古い位置に固定され
#                              "Error in scan matching" で詰まるため、再起動して取り直させる。
#   /initialpose             : lama (iris_lama_loc2d) を spawn 位置で再ローカライズさせる。
# latch/transient_local は使わない (helper 再起動時に古いイベントが再配送されて
# 不要な matcher 再起動を誘発するのを避けるため)。
_reset_event_pub = _hsr.create_publisher_reliable(
    '/isaac/reset_world_event', EmptyMsg)
_initialpose_pub = _hsr.create_publisher_reliable(
    '/initialpose', PoseWithCovarianceStamped)


def _publish_localization_reset():
    """reset (テレポート) 後に localization を spawn 位置へ復帰させる。

    時刻はリセットしない (sim time は単調増加のまま)。stamp は現在 sim time を使う。
    巻き戻すと matcher の dt<=0 や TF extrapolation を招くため。
    """
    # spawn 姿勢を /initialpose で lama に通知 (map フレーム)。
    _q = euler_angles_to_quat(
        [0.0, 0.0, float(_robot_spawn['yaw'])])  # (w, x, y, z)
    _ip = PoseWithCovarianceStamped()
    _ip.header.frame_id = 'map'
    _ip.header.stamp = _hsr.get_ros_time(simulation_context.current_time)
    _ip.pose.pose.position.x = float(_robot_spawn['x'])
    _ip.pose.pose.position.y = float(_robot_spawn['y'])
    _ip.pose.pose.position.z = 0.0
    _ip.pose.pose.orientation.w = float(_q[0])
    _ip.pose.pose.orientation.x = float(_q[1])
    _ip.pose.pose.orientation.y = float(_q[2])
    _ip.pose.pose.orientation.z = float(_q[3])
    # 対角のみ小さめの分散 (要素 0,7,35 が x,y,yaw)。
    _cov = [0.0] * 36
    _cov[0] = 0.01
    _cov[7] = 0.01
    _cov[35] = 0.02
    _ip.pose.covariance = _cov
    _initialpose_pub.publish(_ip)

    # matcher 再起動ヘルパーへ通知。
    _reset_event_pub.publish(EmptyMsg())


def _reset_objects():
    """spawn した全動的物体を初期姿勢へ戻し、速度をゼロにする。

    把持コード (hsr.py の attach-grasp) と同じく dc.set_rigid_body_pose +
    速度ゼロを使う。眠っている body はテレポートを無視するので、pose 設定の後に
    wake_up_rigid_body で起こす。
    """
    for s in _spawn_initial_states:
        h = _hsr.dc.get_rigid_body(s['body_path'])
        if not h:
            continue
        t = _dynamic_control.Transform()
        t.p = s['p']
        # DC の Transform.r は (x, y, z, w) 順。保存は (w, x, y, z)。
        t.r = (s['q'][1], s['q'][2], s['q'][3], s['q'][0])
        _hsr.dc.set_rigid_body_pose(h, t)
        _hsr.dc.set_rigid_body_linear_velocity(h, (0.0, 0.0, 0.0))
        _hsr.dc.set_rigid_body_angular_velocity(h, (0.0, 0.0, 0.0))
        _hsr.dc.wake_up_rigid_body(h)


# disable showing lidar beam
_lidar_path = '/hsrb/hsrb/base_range_sensor_link/Lidar'
_lidar_prim = omni.usd.get_context().get_stage().GetPrimAtPath(_lidar_path)
if _lidar_prim.IsValid():
    _draw_attr = _lidar_prim.GetAttribute('drawLines')
    if _draw_attr and _draw_attr.IsValid():
        _draw_attr.Set(False)
        print(f'[sample-ros] disable showing lidar beam: {_lidar_path}')

_physics_step_index = 0
while kit.is_running():
    # Run with a fixed step size
    _render_this_step = (_physics_step_index % RENDER_EVERY_N_STEPS == 0)
    if _num_people > 0:
        # 人 (UsdSkel) のアニメーションは kit.update() を回さないと評価されない。
        # ただし step(render=True) は内部で描画するので、その後に kit.update()
        # を足すと「1 コマで 2 回描画」になりレンダラが不安定になる
        # (X 接続断・セグフォルトの原因)。そこで物理ステップは描画なし
        # (render=False) にし、描画とアニメ評価は kit.update() の 1 回に任せる。
        # その kit.update() も RENDER_EVERY_N_STEPS で間引く。以前は毎ステップ
        # 呼んでいたため、人がいるシーンだけ 60 Hz 描画になって間引き設定が丸ごと
        # 無視され、RTF が大きく落ちていた。アニメ評価も同じ間引きになる
        # (既定 2 なら 30 Hz) が、見た目には分からない。
        simulation_context.step(render=False)
        if _render_this_step:
            kit.update()
    else:
        # 物理と全身制御は 60 Hz のまま。Kit のビューポートとカメラのレンダープロダクトを
        # 同じ 60 Hz で描くと sim 時間が実時間に対して大きく遅れるので、N ステップに
        # 1 回だけ描く。
        simulation_context.step(render=_render_this_step)
    _physics_step_index += 1
    try:
        _hsr.step()
    except Exception:
        # ここで例外が抜けるとメインループ全体が死に、Sim の ROS 制御
        # (全アクション/サービス) が永久に沈黙してロボットが未制御のまま
        # 漂流する (実際に発生)。1 ステップ分の制御エラーはログして続行する。
        import traceback

        traceback.print_exc()

    # --- 競技モード: 観戦カメラで録画し、競技時間が来たら保存して終了する ---
    if _recorder is not None:
        try:
            if _recorder.step(simulation_context.current_time):
                print('[task] 競技時間終了。動画を保存してシミュレータを終了します。',
                      flush=True)
                _recorder.close()
                _recorder = None
                break
        except Exception:
            import traceback

            traceback.print_exc()

    # --- 調整モード: GUI で動かしたカメラの値を recordings/tune/ に書き出す ---
    if _tuner is not None:
        try:
            _tuner.step()
        except Exception:
            import traceback

            traceback.print_exc()

    # --- reset_world: サービス要求があれば物理ステップ間でここで適用する ---
    if _reset_requested:
        try:
            # 先に USD の姿勢 (手で動かした静的家具・人・物体の親 Xform) を戻し、
            # その後で _reset_objects() が動的剛体のワールド姿勢を確定させる。
            # 逆順にすると親 Xform の復元が剛体のワールド姿勢をずらしてしまう。
            _restore_manual_xforms()
            _reset_objects()
            _hsr.reset_to_spawn(
                _robot_spawn['x'], _robot_spawn['y'], _robot_spawn['yaw'])
            print('[reset_world] world + robot restored to spawn', flush=True)
            # テレポートで詰まる localization (matcher / lama) を spawn 位置で復帰。
            _publish_localization_reset()
        except Exception:
            import traceback

            traceback.print_exc()
        finally:
            _reset_requested = False
            _reset_done.set()

    # --- [recol] 物体の collider を実行時に再登録(GUIの "Set Dynamic Collider (Convex Hull)"
    #     相当)。spawn時の body(YCBの ArticulationRoot 由来)はロボット(別アーティキュレーション)
    #     と衝突しないが、起動後に setRigidBody を再適用すると body が作り直されてロボットと
    #     衝突するようになる(ユーザがGUIで確認)。これが「指が物体をすり抜ける」の根本原因。
    #     指の collider は薄いまま(convexHull)でよい。 ---
    try:
        _rc = globals().get('_recol_step', 0) + 1
        globals()['_recol_step'] = _rc
        if _rc == 120 and not globals().get('_recol_done', False):
            globals()['_recol_done'] = True
            import omni.usd as _ou3
            from pxr import UsdPhysics as _UP3

            _st4 = _ou3.get_context().get_stage()
            for _p in list(_st4.Traverse()):
                _ps = str(_p.GetPath())
                if _ps.startswith('/hsrb'):
                    continue
                # 引き出し (PrismaticJoint を張った trofast) は除外する。
                # setRigidBody は PhysX の剛体を作り直すため、張ったジョイントが
                # 外れて引き出しが落ちる。引き出しは _setup_drawers() の時点で
                # ArticulationRootAPI を外し済み ([recol] が直したかった「指が
                # すり抜ける」原因そのもの) なので、再登録しなくてよい。
                if _ps in _drawer_body_paths:
                    continue
                # /body (YCB/焼き込み) と、生オブジェクトの spawn root
                # (_runtime_rigid_object_paths に記録) の両方を再登録対象にする。
                if _p.HasAPI(_UP3.RigidBodyAPI) and (
                    _ps.endswith('/body') or _ps in _runtime_rigid_object_paths
                ):
                    try:
                        from pxr import PhysxSchema as _PX3

                        # 再登録前の質量を読む(再登録で 0 にリセットされ浮くのを防ぐため)
                        _m0 = None
                        if _p.HasAPI(_UP3.MassAPI):
                            _m0 = _UP3.MassAPI(_p).GetMassAttr().Get()
                        physx_utils.setRigidBody(_p, 'convexHull', False)
                        _rbapi = _PX3.PhysxRigidBodyAPI.Apply(_p)
                        _rbapi.CreateDisableGravityAttr(False)
                        _rbapi.CreateSleepThresholdAttr(0.0)
                        # 質量を復元(再登録後 0 だと重力が効かず浮く)。元が無/0なら 0.3kg。
                        _mapi = _UP3.MassAPI.Apply(_p)
                        _m1 = _mapi.GetMassAttr().Get()
                        _mset = _m0 if (_m0 and _m0 > 0.0) else 0.3
                        _mapi.CreateMassAttr(float(_mset))
                        print(
                            '[recol] re-applied: %s mass(before=%s afterRB=%s set=%.3f)'
                            % (_ps, _m0, _m1, _mset),
                            flush=True,
                        )
                    except Exception as _e:
                        print('[recol] err %s %r' % (_ps, _e), flush=True)
    except Exception as _e:
        print('[recol] outer err %r' % _e, flush=True)

    # --- [wake] 動的物体を定期的に起こす。recol(setRigidBody)後の body はスリープしやすく、
    #     掴んで静止→眠る→開放しても起きず空中で止まる(落ちない)。USD の sleepThreshold は
    #     生の PhysX body に伝わらないので、dc で明示的に wake する。 ---
    try:
        _wk = globals().get('_wake_step', 0) + 1
        globals()['_wake_step'] = _wk
        if _wk > 150 and _wk % 15 == 0:
            _opaths = globals().get('_obj_body_paths')
            if _opaths is None:
                import omni.usd as _ou5
                from pxr import UsdPhysics as _UP5

                _st5 = _ou5.get_context().get_stage()
                _opaths = []
                for _p in _st5.Traverse():
                    _pp = str(_p.GetPath())
                    # /body (YCB/焼き込み) と生オブジェクトの spawn root の両方を wake 対象に。
                    if (
                        (not _pp.startswith('/hsrb'))
                        and _p.HasAPI(_UP5.RigidBodyAPI)
                        and (_pp.endswith('/body') or _pp in _runtime_rigid_object_paths)
                    ):
                        _opaths.append(_pp)
                globals()['_obj_body_paths'] = _opaths
            for _op in _opaths:
                _h = _hsr.dc.get_rigid_body(_op)
                if _h:
                    _hsr.dc.wake_up_rigid_body(_h)
    except Exception:
        pass


simulation_context.stop()
kit.close()
