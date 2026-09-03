#!/usr/bin/env python3
"""
arena_cameras: アリーナを4方向 (北/南/東/西) から見下ろす競技録画カメラ。

TidyUp 競技で「どこに何が入ったか」を人が目視確認できるように、
部屋の外側の高い位置に固定カメラを 4 台置き、シミュレータ内部で
直接 mp4 に録画する (ROS トピックは使わない)。

使い方:
    make up TIME=600     # 競技時間 600 秒 (シミュレータ内時間)。
                         # 時間が来ると動画を保存してシミュレータが自動終了する。
    make up              # TIME 無し = カメラも録画も無し (通常の開発モード)。
    make tune            # カメラ調整モード。録画せず、GUI で位置・画角を
                         # 見ながら動かして設定値を書き出す (下記)。

出力:
    recordings/YYYYMMDD_HHMMSS/arena.mp4
    (コンテナ内 /recordings がホストのリポジトリ内 recordings/ に bind mount)

    4 画面を 2x2 に並べた 1 本の動画:
        ┌─────────┬─────────┐
        │  NORTH  │  EAST   │
        ├─────────┼─────────┤
        │  SOUTH  │  WEST   │
        └─────────┴─────────┘
    各画面の左下にカメラ名、全体の左上に「TASK 経過時間 / 競技時間」を焼き込む。

動画は 1/FPS シミュレータ秒 = 1 フレームで書き出すので、描画が実時間より
遅くても、動画の再生時間 = シミュレータ内の競技時間になる。

------------------------------------------------------------
設定 (configs/placement.yaml の arena_cameras: セクション、無ければ既定値)
------------------------------------------------------------
書きかたは 2 通りある。

(A) 4台をまとめて自動配置する (簡単。中心のまわりに等間隔で置く)

    arena_cameras:
      center: [2.0, 2.0]   # 注視点 (省略すると床の中心)
      distance: 6.5        # 中心からの水平距離 (m)
      height: 4.5          # カメラの高さ (m)
      look_height: 0.3     # 見る高さ (床から何 m の点を狙うか)
      fov_deg: 70          # 画角 (水平方向の視野角, 度)。広いほど広く写る
      resolution: [640, 480]
      fps: 20

(B) 1台ずつ好きな場所に置く (cameras: があると A の center/distance/height
    より優先される)。make tune で書き出されるのはこの形式。

    arena_cameras:
      resolution: [640, 480]
      fps: 20
      cameras:
        north:
          position: [2.0, 4.6, 3.6]   # カメラの座標
          look_at:  [2.0, 2.0, 0.3]   # 見る点
          fov_deg: 70                 # このカメラだけの画角 (省略可)
        south: {...}
        east:  {...}
        west:  {...}

画角は fov_deg (度) でも focal_length (mm) でも指定できる。両方あれば
fov_deg を優先する。fov_deg が大きい = 広角 = 広く写るが小さく見える。
"""
from __future__ import annotations

import datetime
import math
import os
import time
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import yaml

import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.utils.prims import create_prim
from pxr import Gf, Usd, UsdGeom

DEFAULT_CONFIG_PATH = "/app/configs/placement.yaml"
CAMERA_NAMES = ("north", "south", "east", "west")
CAMERA_ROOT = "/World/ArenaCameras"

# USD カメラの既定の水平アパーチャ (mm)。画角(度) と焦点距離(mm) を
# 相互変換するのに使う。これを固定しておくと fov_deg の意味がぶれない。
APERTURE_MM = 20.955
DEFAULT_FOCAL_LENGTH = 15.0      # 約 70 度相当 (広角ぎみ = 部屋全体が入る)
DEFAULT_LOOK_HEIGHT = 0.3        # 床の少し上を見る

# 調整モードの出力先 (/recordings はホストの recordings/ に繋がっている)
TUNE_DIR = "/recordings/tune"


def log(message: str) -> None:
    print(f"[arena_cameras] {message}", flush=True)


# ============================================================
# 設定の読み込み・画角の変換
# ============================================================
def _load_config() -> Dict[str, Any]:
    """placement.yaml の arena_cameras: セクションを読む (無ければ空)。"""
    candidates = [
        os.environ.get("PLACEMENT_CONFIG", DEFAULT_CONFIG_PATH),
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "configs", "placement.yaml",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
            return cfg.get("arena_cameras") or {}
    return {}


def fov_to_focal(fov_deg: float) -> float:
    """画角 (度) -> 焦点距離 (mm)。"""
    fov_deg = min(max(float(fov_deg), 1.0), 179.0)
    return APERTURE_MM / (2.0 * math.tan(math.radians(fov_deg) / 2.0))


def focal_to_fov(focal_mm: float) -> float:
    """焦点距離 (mm) -> 画角 (度)。ログや yaml のコメント表示用。"""
    focal_mm = max(float(focal_mm), 0.1)
    return math.degrees(2.0 * math.atan(APERTURE_MM / (2.0 * focal_mm)))


def _focal_from(section: Dict[str, Any], fallback: float) -> float:
    """設定の 1 ブロックから焦点距離を決める (fov_deg があればそちら優先)。"""
    if section.get("fov_deg") is not None:
        return fov_to_focal(section["fov_deg"])
    if section.get("focal_length") is not None:
        return float(section["focal_length"])
    return fallback


# ============================================================
# 4台の置き場所を決める
# ============================================================
def resolve_poses(cfg: Dict[str, Any], center_x: float, center_y: float
                  ) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, Any]]:
    """設定から 4 台ぶんの (position, look_at, focal_length) を作る。

    cameras: が書いてあれば 1 台ずつその値を使い、無ければ
    center/distance/height から自動で 4 方向に並べる。
    """
    _c = cfg.get("center")
    if _c and len(_c) >= 2:
        cx, cy = float(_c[0]), float(_c[1])
    else:
        cx, cy = float(center_x), float(center_y)

    distance = float(cfg.get("distance", 6.5))
    height = float(cfg.get("height", 4.5))
    look_z = float(cfg.get("look_height", DEFAULT_LOOK_HEIGHT))
    base_focal = _focal_from(cfg, DEFAULT_FOCAL_LENGTH)

    auto = {
        "north": (cx, cy + distance, height),
        "south": (cx, cy - distance, height),
        "east": (cx + distance, cy, height),
        "west": (cx - distance, cy, height),
    }

    per_camera = cfg.get("cameras") or {}
    poses: Dict[str, Dict[str, Any]] = {}
    for name in CAMERA_NAMES:
        one = per_camera.get(name) or {}
        position = one.get("position") or auto[name]
        look_at = one.get("look_at") or (cx, cy, look_z)
        poses[name] = {
            "position": tuple(float(v) for v in position),
            "look_at": tuple(float(v) for v in look_at),
            "focal_length": _focal_from(one, base_focal),
        }

    meta = {
        "center": (cx, cy),
        "distance": distance,
        "height": height,
        "look_height": look_z,
        "focal_length": base_focal,
        "explicit": bool(per_camera),
    }
    return poses, meta


def _make_camera_prim(path: str, position, look_at, focal_length: float) -> None:
    """position から look_at を見るカメラ prim を作る。

    USD のカメラは「-Z 方向を向き +Y が上」の約束なので、
    LookAt 行列 (world→カメラ) の逆行列がそのままカメラの姿勢になる。

    姿勢は「移動(translate) + 回転(orient)」の 2 段で持たせる。1 個の行列で
    持つと Isaac Sim の Property パネルに生の行列が出て手で直せないが、
    この形なら GUI 上で数値入力もギズモ(矢印)でのドラッグもできる。
    """
    create_prim(path, "Camera")
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)

    cam = UsdGeom.Camera(prim)
    cam.CreateFocalLengthAttr(float(focal_length))
    cam.CreateHorizontalApertureAttr(APERTURE_MM)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))

    eye = Gf.Vec3d(*position)
    target = Gf.Vec3d(*look_at)
    # 真上から真下を見るときは up=+Z と視線が平行になって計算が壊れるので、
    # そのときだけ up を +Y に逃がす。
    horizontal = math.hypot(target[0] - eye[0], target[1] - eye[1])
    up = Gf.Vec3d(0, 0, 1) if horizontal > 1e-4 else Gf.Vec3d(0, 1, 0)

    view = Gf.Matrix4d()
    view.SetLookAt(eye, target, up)
    world = view.GetInverse()
    quat = Gf.Transform(world).GetRotation().GetQuat()

    _set_translate_orient(prim, eye, quat)


def _set_translate_orient(prim, translation: Gf.Vec3d,
                          rotation: Gf.Quatd) -> None:
    """prim の姿勢を「移動 + 回転」の 2 段の xformOp で置き直す。

    注意: Isaac Sim の create_prim は先に xformOp:translate / xformOp:orient /
    xformOp:scale を作っており、orient は double 型 (quatd) になっている。
    一方 AddOrientOp() の既定は float 型 (quatf) なので、そのまま呼ぶと
    「同じ名前だが型が違う」状態になり、戻り値が無効な XformOp になる。
    無効な XformOp に .Set() するとセグフォルトでシミュレータごと落ちる。
    そこで、いったん既存の xformOp:* を消してから型を明示して作り直す。
    """
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    for name in [a.GetName() for a in prim.GetAttributes()]:
        if name.startswith("xformOp:"):
            prim.RemoveProperty(name)

    translate_op = xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    orient_op = xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    # scale は使わないが、無いと描画側が
    # 「cannot find xform op xformOp:scale」を毎回警告するので等倍で置いておく。
    scale_op = xf.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble)
    if not translate_op or not orient_op or not scale_op:
        # ここで弾いておかないと下の Set() でプロセスごと落ちる
        raise RuntimeError(
            f"{prim.GetPath()}: xformOp を作れませんでした "
            f"(translate={bool(translate_op)}, orient={bool(orient_op)}, "
            f"scale={bool(scale_op)})")
    translate_op.Set(Gf.Vec3d(translation))
    orient_op.Set(Gf.Quatd(rotation))
    scale_op.Set(Gf.Vec3d(1.0, 1.0, 1.0))


def create_cameras(poses: Dict[str, Dict[str, Any]]) -> Dict[str, str]:
    """4台のカメラ prim を作り、{名前: prim パス} を返す。"""
    paths = {}
    for name in CAMERA_NAMES:
        pose = poses[name]
        path = f"{CAMERA_ROOT}/Camera_{name}"
        _make_camera_prim(path, pose["position"], pose["look_at"],
                          pose["focal_length"])
        paths[name] = path
        log(f"camera {name}: pos={_fmt_vec(pose['position'])} "
            f"look_at={_fmt_vec(pose['look_at'])} "
            f"画角 {focal_to_fov(pose['focal_length']):.0f}度")
    return paths


# ============================================================
# GUI で動かしたカメラの現在値を読み戻す
# ============================================================
def read_camera_pose(path: str, look_z: float = DEFAULT_LOOK_HEIGHT
                     ) -> Optional[Dict[str, Any]]:
    """ステージ上のカメラの「今の」位置・視線・画角を読む。

    GUI でギズモを動かしても数値を打ち込んでも、結果はワールド変換行列に
    出るので、そこから位置と向きを取り出す。向きは「視線(-Z 方向)が
    z=look_z の高さの平面と交わる点」を look_at として書き戻す。
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None

    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    position = matrix.ExtractTranslation()
    forward = matrix.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    right = matrix.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()

    if forward[2] < -1e-3:                       # 下を向いている = 床と交わる
        t = (look_z - position[2]) / forward[2]
        t = min(max(t, 0.5), 50.0)
    else:                                        # 水平・上向きなら 5m 先を見る
        t = 5.0
    look_at = position + forward * t

    focal = UsdGeom.Camera(prim).GetFocalLengthAttr().Get()
    focal = float(focal) if focal else DEFAULT_FOCAL_LENGTH

    return {
        "position": (position[0], position[1], position[2]),
        "look_at": (look_at[0], look_at[1], look_at[2]),
        "focal_length": focal,
        # 右方向が水平から傾いていたら「首をかしげている」= roll がある。
        # position/look_at 形式では表せないので警告に使う。
        "roll": abs(right[2]),
    }


def format_config_yaml(poses: Dict[str, Dict[str, Any]],
                       resolution, fps: float) -> str:
    """読み戻した 4 台ぶんを placement.yaml に貼れる形の文字列にする。"""
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        "# ============================================================",
        "# GUI で調整した観戦カメラの設定 (make tune が自動生成)",
        f"# 生成時刻: {stamp}",
        "#",
        "# 使い方: 下の arena_cameras: ブロックをまるごとコピーして",
        "#         configs/placement.yaml の arena_cameras: と差し替える。",
        "# ============================================================",
        "arena_cameras:",
        f"  resolution: [{int(resolution[0])}, {int(resolution[1])}]",
        f"  fps: {fps:g}",
        "",
        "  # 1台ずつ置いた4台 (position から look_at を見る)。",
        "  # このブロックがあると center/distance/height は使われない。",
        "  cameras:",
    ]
    for name in CAMERA_NAMES:
        pose = poses[name]
        fov = focal_to_fov(pose["focal_length"])
        lines += [
            f"    {name}:",
            f"      position: {_fmt_vec(pose['position'])}",
            f"      look_at: {_fmt_vec(pose['look_at'])}",
            f"      fov_deg: {fov:.1f}",
        ]
    return "\n".join(lines) + "\n"


def _fmt_vec(vec) -> str:
    return "[" + ", ".join(f"{float(v):.3f}" for v in vec) + "]"


def _format_time(sec: float) -> str:
    sec = max(0, int(sec))
    return f"{sec // 60:02d}:{sec % 60:02d}"


# ============================================================
# 画像の合成 (録画とプレビューで共通)
# ============================================================
def _put_text(img, text, pos, scale, outline, inner) -> None:
    """黒フチ + 白文字。どんな背景でも読めるようにする。"""
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 0, 0), outline, cv2.LINE_AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, (255, 255, 255), inner, cv2.LINE_AA)


def _tile(annotator, name: str, width: int, height_px: int) -> np.ndarray:
    """1 カメラ分の画像 (BGR) を返す。まだ描画が無ければ黒画面。"""
    data = annotator.get_data()
    if data is None or getattr(data, "size", 0) == 0:
        img = np.zeros((height_px, width, 3), dtype=np.uint8)
    else:
        img = cv2.cvtColor(np.asarray(data)[:, :, :3], cv2.COLOR_RGB2BGR)
        if img.shape[0] != height_px or img.shape[1] != width:
            # 解像度が想定と違うと後段の連結で落ちるので念のため揃える
            img = cv2.resize(img, (width, height_px))
    _put_text(img, name.upper(), (10, height_px - 12), 0.7, 4, 2)
    return img


def compose_grid(annotators: Dict[str, Any], width: int,
                 height_px: int) -> np.ndarray:
    """4 画面を 2x2 に合成: 上段 north|east, 下段 south|west。"""
    top = np.hstack([_tile(annotators["north"], "north", width, height_px),
                     _tile(annotators["east"], "east", width, height_px)])
    bottom = np.hstack([_tile(annotators["south"], "south", width, height_px),
                        _tile(annotators["west"], "west", width, height_px)])
    return np.vstack([top, bottom])


def _attach_annotators(paths: Dict[str, str], width: int,
                       height_px: int) -> Dict[str, Any]:
    """各カメラに「絵を取り出す口」(annotator) を繋ぐ。"""
    annotators = {}
    for name, path in paths.items():
        render_product = rep.create.render_product(path, (width, height_px))
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach([render_product])
        annotators[name] = annot
    return annotators


# ============================================================
# 録画 (競技モード)
# ============================================================
class ArenaRecorder:
    """4方向カメラの録画係。

    setup() でカメラを作り、メインループから step(sim_time) を毎回呼ぶ。
    競技時間に達すると step() が True を返すので、呼び出し側は録画を
    close() してシミュレータを終了する。
    """

    def __init__(self, task_time_sec: float,
                 center_x: float = 0.0, center_y: float = 0.0,
                 out_root: str = "/recordings"):
        cfg = _load_config()
        self.task_time = float(task_time_sec)
        self.poses, self.meta = resolve_poses(cfg, center_x, center_y)
        self.out_root = out_root

        resolution = cfg.get("resolution") or [640, 480]
        self.width, self.height_px = int(resolution[0]), int(resolution[1])
        self.fps = float(cfg.get("fps", 20))

        self.annotators = {}   # name -> rgb annotator
        self.writer = None     # 2x2 合成した 1 本の動画の VideoWriter
        self.frame_count = 0
        self.out_dir = None
        self._t0 = None        # 競技開始のシミュレータ時刻 (最初の step で決まる)
        self._next_capture = 0.0

    # ------------------------------------------------------------
    def setup(self) -> None:
        paths = create_cameras(self.poses)
        self.annotators = _attach_annotators(paths, self.width, self.height_px)

        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.join(self.out_root, stamp)
        os.makedirs(self.out_dir, exist_ok=True)
        log(f"録画開始: 競技時間 {_format_time(self.task_time)} "
            f"-> {self.out_dir}/arena.mp4 (4画面を2x2に合成)")

    # ------------------------------------------------------------
    def step(self, sim_time: float) -> bool:
        """毎ループ呼ぶ。競技時間に達したら True を返す。"""
        if self._t0 is None:
            self._t0 = sim_time
        elapsed = sim_time - self._t0

        # 1/fps シミュレータ秒ごとに 1 フレーム取り込む
        if elapsed >= self._next_capture:
            self._capture(elapsed)
            self._next_capture += 1.0 / self.fps

        return elapsed >= self.task_time

    def _capture(self, elapsed: float) -> None:
        grid = compose_grid(self.annotators, self.width, self.height_px)

        # 全体の左上にタスク時間
        label = f"TASK {_format_time(elapsed)} / {_format_time(self.task_time)}"
        _put_text(grid, label, (10, 38), 1.1, 5, 2)

        if self.writer is None:
            path = os.path.join(self.out_dir, "arena.mp4")
            self.writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps, (grid.shape[1], grid.shape[0]))
        self.writer.write(grid)
        self.frame_count += 1

    # ------------------------------------------------------------
    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()
            log(f"saved arena.mp4 ({self.frame_count} frames, "
                f"{self.width * 2}x{self.height_px * 2})")
        else:
            log("WARNING: フレームが 1 枚も取れませんでした")
        log(f"録画完了: {self.out_dir}")


# ============================================================
# 調整モード (make tune)
# ============================================================
class ArenaCameraTuner:
    """GUI で 4 台のカメラを動かし、その値を yaml に書き出す係。

    録画はしない。代わりに、
      - カメラ prim を作る (GUI の Viewport から覗ける・動かせる)
      - 動かすたびに recordings/tune/arena_cameras.yaml を書き直す
      - 実際に録画される 2x2 の絵を recordings/tune/preview.png に出す
    """

    # 何秒ごとに「動いたか」を見に行くか (毎フレーム読むと重いので間引く)
    CHECK_INTERVAL_SEC = 0.5

    def __init__(self, center_x: float = 0.0, center_y: float = 0.0,
                 out_dir: str = TUNE_DIR):
        cfg = _load_config()
        self.poses, self.meta = resolve_poses(cfg, center_x, center_y)
        self.out_dir = out_dir

        resolution = cfg.get("resolution") or [640, 480]
        self.width, self.height_px = int(resolution[0]), int(resolution[1])
        self.fps = float(cfg.get("fps", 20))

        self.paths: Dict[str, str] = {}
        self.annotators: Dict[str, Any] = {}
        self._last_check = 0.0
        self._last_signature = None
        self._preview_ready = False   # 中身のあるプレビューを 1 回でも書いたか
        self._warned_roll = False

    # ------------------------------------------------------------
    def setup(self) -> None:
        self.paths = create_cameras(self.poses)
        self.annotators = _attach_annotators(
            self.paths, self.width, self.height_px)
        os.makedirs(self.out_dir, exist_ok=True)

        fov = focal_to_fov(self.meta["focal_length"])
        log("=" * 60)
        log("カメラ調整モード (録画はしません)")
        log("")
        log("  1. Isaac Sim の Viewport 左上のカメラ選択 (Perspective と")
        log("     書いてある所) から Camera_north などを選ぶと、その")
        log("     カメラの見え方がそのまま映る。")
        log("  2. その状態でマウス/WASD で視点を動かすと、カメラ自体が動く。")
        log("     Stage ツリーで /World/ArenaCameras/Camera_north を選び、")
        log("     Property パネルの Translate に数値を打ち込んでもよい。")
        log("  3. 画角は Property > Camera > Focal Length で変えられる")
        log(f"     (今は {self.meta['focal_length']:.1f}mm = 約{fov:.0f}度。"
            f"小さいほど広角)。")
        log("  4. 動かすたびに設定が下のファイルに書き出される:")
        log(f"       {self.out_dir}/arena_cameras.yaml   <- placement.yaml に貼る")
        log(f"       {self.out_dir}/preview.png          <- 実際に録画される2x2の絵")
        log("")
        log(f"  録画解像度は 1台あたり {self.width}x{self.height_px}。")
        log("  Viewport の縦横比がこれと違うと写る範囲が少しずれるので、")
        log("  最終確認は preview.png で行うこと。")
        log("=" * 60)

        self._write_outputs()   # 起動直後の状態も一度書いておく

    # ------------------------------------------------------------
    def step(self) -> None:
        """メインループから毎回呼ぶ。動いていたらファイルを書き直す。"""
        now = time.time()
        if now - self._last_check < self.CHECK_INTERVAL_SEC:
            return
        self._last_check = now
        self._write_outputs()

    # ------------------------------------------------------------
    def _have_frames(self) -> bool:
        """4台とも実際に絵が届いているか (起動直後は届いていない)。"""
        for annot in self.annotators.values():
            data = annot.get_data()
            if data is None or getattr(data, "size", 0) == 0:
                return False
        return True

    # ------------------------------------------------------------
    def _write_outputs(self) -> None:
        current = {}
        for name, path in self.paths.items():
            pose = read_camera_pose(path, self.meta["look_height"])
            if pose is None:
                return          # まだステージに無い / 消された
            current[name] = pose

        # 何も動いていなければ書き直さない (無駄な書き込みを避ける)。
        # ただし起動直後はまだ描画が来ておらずプレビューが真っ黒なので、
        # 中身のある絵が撮れるまでは動いていなくても書き直す。
        signature = tuple(
            (round(v, 4) for name in CAMERA_NAMES
             for v in (*current[name]["position"], *current[name]["look_at"],
                       current[name]["focal_length"]))
        )
        moved = signature != self._last_signature
        have_frames = self._have_frames()
        if not moved and (self._preview_ready or not have_frames):
            return
        self._last_signature = signature

        text = format_config_yaml(current, (self.width, self.height_px),
                                  self.fps)
        with open(os.path.join(self.out_dir, "arena_cameras.yaml"), "w") as f:
            f.write(text)

        grid = compose_grid(self.annotators, self.width, self.height_px)
        cv2.imwrite(os.path.join(self.out_dir, "preview.png"), grid)
        self._preview_ready = have_frames

        if moved:
            log("カメラ位置を更新 -> " + " / ".join(
                f"{name}:{_fmt_vec(current[name]['position'])}"
                for name in CAMERA_NAMES))
        elif have_frames:
            log(f"プレビュー準備完了: {self.out_dir}/preview.png")

        rolled = [n for n in CAMERA_NAMES if current[n]["roll"] > 0.02]
        if rolled and not self._warned_roll:
            self._warned_roll = True
            log(f"WARNING: {', '.join(rolled)} が横に傾いています (roll)。"
                "position/look_at 形式では傾きを保存できないため、"
                "yaml を適用すると水平に戻ります。")
