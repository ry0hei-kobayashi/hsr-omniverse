#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
object_placement: 選択されたplacement YAMLに従って家具や床へ物体を配置する。

generate_wrs_task のランダム配置を「置き換える」モジュール。
world ファイルから各家具 (unit_box) の天板の高さを計算し、YAML で指定された
物体をその天板の少し上にスポーンして、物理で自然に着地させる。

公開:
  - apply_placements(world_file, drop_func, config_path=None):
        設定ファイルを読み、家具ごとに物体を drop_func で配置する。
  - load_config(path):   YAML を辞書として読み込む (デバッグ用)
  - read_furniture(world_file): 家具名 -> 形状 を返す (デバッグ用)

drop_func は launch_isaacsim.py の drop_object と同じシグネチャを想定:
    drop_func(gazebo_name, name, x, y, z, yaw=0.0, roll=0.0, pitch=0.0)
"""
from __future__ import annotations

import math
import os
import re
import xml.etree.ElementTree as ET
from typing import Any, Callable, Dict, List

import yaml


# ============================================================
# 設定ファイルの場所
# ============================================================
# デフォルトは /app/configs/placement.yaml (docker-compose で bind mount)。
# 物体配置だけを別ファイルへ切り替える場合は OBJECT_PLACEMENT_CONFIG を使う。
# 従来の PLACEMENT_CONFIG も後方互換として残す。

DEFAULT_CONFIG_PATH: str = "/app/configs/placement.yaml"
CONFIG_PATH: str = (
    os.environ.get("OBJECT_PLACEMENT_CONFIG")
    or os.environ.get("PLACEMENT_CONFIG")
    or DEFAULT_CONFIG_PATH
)


def log(message: str) -> None:
    print(f"[placement] {message}")


# ============================================================
# YAML 読み込み
# ============================================================

def load_config(path: str) -> Dict[str, Any]:
    """placement.yaml を辞書として読み込む。

    指定パスが無ければ、このファイルから見た repo 内の configs/placement.yaml
    を探す (ホストで直接実行したときのフォールバック)。
    """
    if not os.path.exists(path):
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        fallback = os.path.join(repo_root, "configs", "placement.yaml")
        if os.path.exists(fallback):
            path = fallback
        else:
            log(f"WARNING: config not found: {path} (nor {fallback})")
            return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    log(f"loaded config: {path}")
    return data


# ============================================================
# world ファイルから家具の形状を読む
# ============================================================

# 名前を「頭の部分」と「末尾の段番号」に分ける正規表現。
#   "cabinet_3"  -> base="cabinet", idx="3"
#   "shelf_0"    -> base="shelf",   idx="0"
#   "dining_table" や "dining_wall_0_" (末尾が数字でない) -> マッチしない
_TIER_RE = re.compile(r"^(.+)_(\d+)$")


# ============================================================
# 既知の USD 家具モデルの「置ける面」の定義
# ============================================================
# unit_box の家具は world の scale から天板を計算できるが、本物の USD モデル
# (wrc_* など) は形状が world に書かれていない。そこで、モデルごとの
# 「物を置ける面の高さ (モデル原点からの z, 低い順)」をここに定義しておく。
# 値は本家 Gazebo モデル (tmc_wrs_gazebo_worlds/models/*/model.sdf) の
# 衝突ボックスの天面から取った実寸。
#
#   tops : 各段の天面の高さ [m] (モデル原点=床基準)。並び順が tier 番号になる。
#   size : 水平方向のおおよその footprint (x, y) [m]。部屋の外周計算に使う。
KNOWN_MODEL_SURFACES: Dict[str, Dict[str, Any]] = {
    # 本棚: 板の天面。tier 0=一番下の棚, 4=一番上 (高さ2.02mでHSRには高すぎ注意)
    "wrc_bookshelf": {"tops": [0.06, 0.50, 0.80, 1.05, 2.02], "size": (0.80, 0.28)},
    # 長机: 天板 1 枚 (1.2 x 0.4 m)
    "wrc_long_table": {"tops": [0.40], "size": (1.20, 0.40)},
    # 高い机: 天板 1 枚 (0.4 x 0.4 m)
    "wrc_tall_table": {"tops": [0.60], "size": (0.40, 0.40)},
    # 部屋の壁 (置く対象ではないが、部屋の外周計算に使う)
    "wrc_frame": {"tops": [], "size": (6.1, 4.2)},
}

# placement.yaml の furniture: で後から配置する、ローカル USD 家具の置き面。
# world ファイルの include ではないため、上の KNOWN_MODEL_SURFACES とは別に
# USD パスごとの実寸を定義する。値は家具 USD の原寸 (scale=1.0) の天板高さ。
CONFIG_FURNITURE_SURFACES: Dict[str, Dict[str, Any]] = {
    "restaurant/round_table/model.usd": {
        "tops": [0.49],
    },
}


def read_config_furniture(cfg: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """placement.yaml の furniture: から物を置ける家具を読む。

    furniture_spawn は world ファイルの読み込み後に家具を生成するため、従来の
    read_furniture() だけでは furniture: の家具を placements: の置き台にできなかった。
    ここでは既知のローカル USD 家具について、YAML の位置・向き・scale から
    object_placement 用の家具情報を作る。
    """
    section = cfg.get("furniture") or []
    items = section.get("list") or [] if isinstance(section, dict) else section
    result: Dict[str, Dict[str, Any]] = {}
    for index, raw in enumerate(items):
        if not isinstance(raw, dict):
            continue
        usd = str(raw.get("usd", ""))
        spec = CONFIG_FURNITURE_SURFACES.get(usd)
        if spec is None:
            continue
        try:
            name = str(raw.get("name", f"furniture_{index}"))
            x = float(raw["x"])
            y = float(raw["y"])
            z = float(raw.get("z", 0.0))
            yaw = math.radians(float(raw.get("yaw", 0.0)))
            scale = float(raw.get("scale", 1.0))
        except (KeyError, TypeError, ValueError):
            log(f"WARNING: furniture[{index}] の配置情報を読めません。スキップ。")
            continue
        result[name] = {
            "x": x,
            "y": y,
            "yaw": yaw,
            "tops": [z + top * scale for top in spec["tops"]],
        }
    return result


def read_furniture(world_file: str) -> Dict[str, Dict[str, Any]]:
    """world ファイルの <include> から家具を読み、

        家具名 -> {"x", "y", "yaw", "tops": [低い段の天面, ..., 高い段の天面]}

    の辞書を返す。"tops" は各段の天板の高さ (z + scale_z/2) を低い順に並べたもので、
    その並び順がそのまま tier 番号 (0 = 一番下の段) になる。

    同じ頭の名前 (例: cabinet_0..3) で、かつ同じ位置 (x, y) に積み重なっている箱は
    1 つの多段家具 "cabinet" としてまとめる。位置がバラバラなもの (壁など) は
    段とはみなさず、それぞれフルネームで 1 段だけの家具として登録する。

    scale を持たない include (= 本物の model.usd 家具) は、モデル名が
    KNOWN_MODEL_SURFACES にあればその定義の tops を使って登録する
    (include の名前がそのまま家具名になる)。無ければスキップ。
    """
    tree = ET.parse(world_file)
    root = tree.getroot()

    # --- まず world 内の箱を全部読む (USD 家具は known_usd に別で集める) ---
    boxes: List[Dict[str, Any]] = []
    known_usd: Dict[str, Dict[str, Any]] = {}
    for inc in root.findall("world/include"):
        name_tag = inc.find("name")
        pose_tag = inc.find("pose")
        scale_tag = inc.find("scale")
        uri_tag = inc.find("uri")
        if name_tag is None or pose_tag is None:
            continue
        pose = [float(n) for n in pose_tag.text.split()]
        x, y, z = pose[0], pose[1], pose[2]
        yaw = pose[5]              # pose は x y z roll pitch yaw
        if scale_tag is None:
            # 本物の USD 家具: 既知モデルなら定義済みの棚板高さで登録する。
            uri = (uri_tag.text if uri_tag is not None else "").replace("model://", "")
            spec = KNOWN_MODEL_SURFACES.get(uri)
            if spec and spec["tops"]:
                known_usd[name_tag.text] = {
                    "x": x, "y": y, "yaw": yaw,
                    # include の z (通常 0) を足して world 座標の天面にする
                    "tops": [z + t for t in spec["tops"]],
                }
            continue
        scale = [float(n) for n in scale_tag.text.split()]
        top = z + scale[2] / 2.0  # この箱の天面 (= 段の置ける面)
        boxes.append({"name": name_tag.text, "x": x, "y": y, "yaw": yaw, "top": top})

    # --- 名前の「頭の部分」でグループ分け ---
    groups: Dict[str, List[Dict[str, Any]]] = {}
    for b in boxes:
        m = _TIER_RE.match(b["name"])
        base = m.group(1) if m else b["name"]
        groups.setdefault(base, []).append(b)

    # --- グループごとに、多段家具か単段家具かを判定して登録 ---
    furniture: Dict[str, Dict[str, Any]] = {}
    for base, members in groups.items():
        # 同じ位置 (x, y) に積み重なっているか? (小数 3 桁で比較)
        footprints = {(round(b["x"], 3), round(b["y"], 3)) for b in members}
        if len(members) > 1 and len(footprints) == 1:
            # 多段家具: 天面を低い順に並べて tops にする
            members.sort(key=lambda b: b["top"])
            ref = members[0]
            furniture[base] = {
                "x": ref["x"], "y": ref["y"], "yaw": ref["yaw"],
                "tops": [b["top"] for b in members],
            }
        else:
            # 段ではない: それぞれフルネームで 1 段だけの家具として登録
            for b in members:
                furniture[b["name"]] = {
                    "x": b["x"], "y": b["y"], "yaw": b["yaw"],
                    "tops": [b["top"]],
                }

    # --- 本物の USD 家具 (KNOWN_MODEL_SURFACES 定義) を追加登録 ---
    # include の名前 (例: wrc_bookshelf, wrc_long_table_0) がそのまま家具名。
    furniture.update(known_usd)
    return furniture


def world_xy_bounds(world_file: str):
    """world 内の家具・壁の XY 外周を返す。

        (min_x, max_x, min_y, max_y)

    unit_box は scale から、本物の USD 家具は KNOWN_MODEL_SURFACES の
    footprint (size) から、各々回転 (yaw) を考慮した四隅で算出する。
    何も無ければ None。床・背景幕のサイズ/中心の算出に使う。
    """
    tree = ET.parse(world_file)
    root = tree.getroot()
    xs: List[float] = []
    ys: List[float] = []
    for inc in root.findall("world/include"):
        pose_tag = inc.find("pose")
        scale_tag = inc.find("scale")
        uri_tag = inc.find("uri")
        if pose_tag is None:
            continue
        pose = [float(n) for n in pose_tag.text.split()]
        x, y, yaw = pose[0], pose[1], pose[5]
        if scale_tag is not None:
            scale = [float(n) for n in scale_tag.text.split()]
            sx, sy = scale[0], scale[1]
        else:
            # USD 家具: 既知モデルなら footprint を使う
            uri = (uri_tag.text if uri_tag is not None else "").replace("model://", "")
            spec = KNOWN_MODEL_SURFACES.get(uri)
            if spec is None:
                continue
            sx, sy = spec["size"]
        for dx in (-sx / 2.0, sx / 2.0):
            for dy in (-sy / 2.0, sy / 2.0):
                xs.append(x + dx * math.cos(yaw) - dy * math.sin(yaw))
                ys.append(y + dx * math.sin(yaw) + dy * math.cos(yaw))
    if not xs:
        return None
    return (min(xs), max(xs), min(ys), max(ys))


# ============================================================
# 配置本体
# ============================================================

def _parse_item(item: Any) -> Dict[str, Any]:
    """YAML の 1 エントリを正規化する。

    文字列なら物体名だけ、辞書なら object/dx/dy/yaw/roll/pitch を読む。
    """
    if isinstance(item, str):
        return {"object": item, "dx": 0.0, "dy": 0.0,
                "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "tier": 0}
    return {
        # compe用ファイルでは場所だけを「スロット」として定義できる。
        # 通常モードではapply_placements側でobject必須を検証する。
        "object": item.get("object"),
        "dx": float(item.get("dx", 0.0)),
        "dy": float(item.get("dy", 0.0)),
        "yaw": float(item.get("yaw", 0.0)),
        "roll": float(item.get("roll", 0.0)),
        "pitch": float(item.get("pitch", 0.0)),
        "tier": int(item.get("tier", 0)),  # 段番号 (0 = 一番下)。省略時は一番下。
    }


def _parse_floor_item(item: Any) -> Dict[str, Any]:
    """floor リストの 1 エントリを正規化する。

    家具置き (_parse_item) は家具中心からのズレ dx/dy を読むが、床置きは
    床 (z=0) の絶対座標 x/y を読む。x/y は必須なので辞書で書く必要がある。
    """
    return {
        "object": item.get("object"),
        "x": float(item["x"]),
        "y": float(item["y"]),
        "z": float(item.get("z", 0.0)),  # 省略時 0=床。机の天板等に載せるなら高さを指定。
        "yaw": float(item.get("yaw", 0.0)),
        "roll": float(item.get("roll", 0.0)),
        "pitch": float(item.get("pitch", 0.0)),
    }


def apply_placements(
    world_file: str,
    drop_func: Callable[..., None],
    config_path: str | None = None,
) -> int:
    """placement.yaml に従って物体を配置する。配置した個数を返す。

    Args:
        world_file: 家具の位置・大きさが書かれた .world ファイルのパス
        drop_func:  物体をスポーンする関数 (launch_isaacsim.py の drop_object)
        config_path: 設定ファイル。省略時は CONFIG_PATH。
    """
    path = config_path or CONFIG_PATH
    cfg = load_config(path)
    # placement.yaml は robot / objects / people の 3 セクション構成。
    # 物体配置はそのうち objects: セクションを見る。
    objects_cfg = cfg.get("objects") or {}
    placements = objects_cfg.get("placements") or {}
    clearance = float(objects_cfg.get("drop_clearance", 0.05))

    if not placements and not objects_cfg.get("floor") and not objects_cfg.get("obstacles"):
        log("WARNING: 'placements' / 'floor' / 'obstacles' が空です。配置する物体がありません。")
        return 0

    furniture = read_furniture(world_file)
    # furniture: で定義された既知の USD 家具も placements: の置き台にする。
    # 同名の world 家具がある場合は、world 側の定義を優先する。
    for name, info in read_config_furniture(cfg).items():
        furniture.setdefault(name, info)

    requested = 0   # 設定で要求された物体数
    placed = 0      # 実際に配置できた数
    failed = []     # 見つからなかった物体名
    for furn_name, items in placements.items():
        if furn_name not in furniture:
            log(f'WARNING: 家具 "{furn_name}" が world に見つかりません。スキップ。')
            continue
        if not items:
            continue
        info = furniture[furn_name]
        fx, fy, fyaw = info["x"], info["y"], info["yaw"]
        tops = info["tops"]  # 低い段から順の天面リスト

        for idx, raw in enumerate(items):
            it = _parse_item(raw)
            obj_name = it["object"]
            if not isinstance(obj_name, str) or not obj_name:
                raise ValueError(f'placements.{furn_name}[{idx}] requires "object"')
            dx, dy = it["dx"], it["dy"]
            tier = it["tier"]

            # tier 番号が段数の範囲外なら配置せず警告
            if tier < 0 or tier >= len(tops):
                log(f'WARNING: 家具 "{furn_name}" に tier={tier} は無効 '
                    f"(段数は 0〜{len(tops) - 1})。{obj_name} をスキップ。")
                requested += 1
                failed.append(f"{furn_name}[tier={tier}]/{obj_name}")
                continue
            top_z = tops[tier]  # 指定した段の天面の高さ

            # 天板中央からのオフセットを家具の向き (fyaw) で回して world 座標へ
            wx = fx + dx * math.cos(fyaw) - dy * math.sin(fyaw)
            wy = fy + dx * math.sin(fyaw) + dy * math.cos(fyaw)
            wz = top_z + clearance

            # prim パスが衝突しないよう一意な名前にする
            gazebo_name = f"{furn_name}__{obj_name}__{idx}"
            requested += 1
            # drop_func は成功で model.usd のパス、失敗 (モデル不在) で None を返す。
            result = drop_func(
                gazebo_name,
                obj_name,
                wx, wy, wz,
                yaw=fyaw + it["yaw"],
                roll=it["roll"],
                pitch=it["pitch"],
            )
            if result:
                placed += 1
            else:
                failed.append(f"{furn_name}/{obj_name}")

    # --- 床に直接置く物体 (家具ではなく world 座標を直接指定) ---
    # placements は家具の天板に乗せるが、floor は床 (z=0) に絶対座標で置く。
    # 例: floor: [{object: ..., x: 2.0, y: 1.0}]
    floor_items = objects_cfg.get("floor") or []
    for idx, raw in enumerate(floor_items):
        it = _parse_floor_item(raw)
        obj_name = it["object"]
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError(f'objects.floor[{idx}] requires "object"')
        wx, wy = it["x"], it["y"]
        wz = it["z"] + clearance  # z=0 で床、z>0 で机の天板など指定高さの上に落として着地。

        gazebo_name = f"floor__{obj_name}__{idx}"
        requested += 1
        result = drop_func(
            gazebo_name,
            obj_name,
            wx, wy, wz,
            yaw=it["yaw"],
            roll=it["roll"],
            pitch=it["pitch"],
        )
        if result:
            placed += 1
        else:
            failed.append(f"floor/{obj_name}")

    # --- 障害物 (得点対象外) ---
    # 座標形式と物理スポーンは floor と同じだが、設定上の役割を分離する。
    # Seedごとに Coffee can / Toy airplane のいずれか1個を候補4位置の1つへ置く。
    obstacle_items = objects_cfg.get("obstacles") or []
    for idx, raw in enumerate(obstacle_items):
        it = _parse_floor_item(raw)
        obj_name = it["object"]
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError(f'objects.obstacles[{idx}] requires "object"')
        wx, wy = it["x"], it["y"]
        wz = it["z"] + clearance

        gazebo_name = f"obstacle__{obj_name}__{idx}"
        requested += 1
        result = drop_func(
            gazebo_name,
            obj_name,
            wx, wy, wz,
            yaw=it["yaw"],
            roll=it["roll"],
            pitch=it["pitch"],
        )
        if result:
            placed += 1
        else:
            failed.append(f"obstacle/{obj_name}")

    log(f"placed {placed}/{requested} objects from {path}")
    if failed:
        log(f"WARNING: モデルが見つからず配置できなかった物体: {failed}")
    return placed
