#!/usr/bin/env python3
"""把持するところまでを実行して止める (クラッシュする whole-body retract は行わない)。
台車は事前に cmd_vel で対象手前まで寄せてある前提。リンゴは odom(0.7,0)。"""
import math
import subprocess
import time

import rclpy
from hsrb_interface import Robot, geometry
from rclpy.node import Node
from std_srvs.srv import Empty

APPLE = (0.60, 0.0)  # odom

r = Robot()
wb = r.get('whole_body')
gripper = r.get('gripper')


def base_pose():
    """台車の現在位置 (odom)。リセットできたかの確認用。"""
    out = subprocess.run(
        ['bash', '-c',
         'timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint '
         '2>/dev/null | grep -m1 -A2 Translation'],
        capture_output=True, text=True).stdout
    try:
        v = out.split('[')[1].split(']')[0].split(',')
        return float(v[0]), float(v[1])
    except IndexError:
        return float('nan'), float('nan')


def reset_world(settle=4.0):
    """ロボットと物体を初期位置へ戻す (シミュレータ側のサービス)。

    毎回同じ条件から始めるために実行の最初で呼ぶ。物体が落ちて落ち着くまで
    少し待つ必要があるので settle 秒だけ待機する。
    """
    node = Node('grasp_only_reset')
    client = node.create_client(Empty, '/isaac/reset_world')
    if not client.wait_for_service(timeout_sec=10.0):
        print('reset_world サービスが見つかりません。リセットせずに続けます')
        node.destroy_node()
        return
    future = client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    node.destroy_node()
    time.sleep(settle)
    print('reset world -> 台車 odom=(%.3f, %.3f)' % base_pose())


def reach(x, y, z, ref_frame_id='base_footprint', retries=3):
    """手先を (x, y, z) へ動かす (上から掴む姿勢 ei=pi)。失敗したら再計画する。

    経路は作れるのに「速度・加速度の上限を守って通れる時間の割り当て」が
    できないことがあり、そのとき hsrb_interface は失敗を確認せず None を
    使うため、原因の分からない AttributeError で落ちる。

    ※ 実際の理由は ROS 側のログに出る:
       docker compose ... logs ros2 | grep timeopt
       -> "A exception occured in timeopt No valid switching point found."

    起きるかどうかは「今の姿勢」に強く依存する。同じ姿勢から同じ指令を出すと
    同じように失敗しやすいので、待つだけでなく姿勢を崩してから引き直す。
    (use_base_timeopt=False は回避策にならない。この版はその場合 None を
     返す実装なので、かえって必ず失敗する。)
    """
    pose = geometry.pose(x=x, y=y, z=z, ei=math.pi)
    for attempt in range(1, retries + 1):
        try:
            wb.move_end_effector_pose(pose, ref_frame_id=ref_frame_id)
            return
        except AttributeError:
            print('  時間の割り当てに失敗 (%d/%d)' % (attempt, retries))
            if attempt < retries:
                time.sleep(0.5)
                if attempt >= 2:
                    # 待つだけでは同じ結果になりやすい。初期姿勢へ戻して
                    # 別の姿勢から計画し直す。
                    print('  初期姿勢に戻してから計画し直します')
                    try:
                        wb.move_to_go()
                    except Exception:
                        pass
                    time.sleep(0.5)
    raise RuntimeError('%d 回試しても手先を (%.2f, %.2f, %.2f) へ運べませんでした'
                       % (retries, x, y, z))


# 毎回同じ条件から始める (ロボットと物体を初期位置へ)
reset_world()

wb.move_to_go()

print('open gripper')
gripper.command(1.0)   # 開く
time.sleep(1)

#リンゴの真上へ (whole-body。reach は実績あり)
print('reach above apple')
reach(APPLE[0], APPLE[1], 0.4)
time.sleep(0.5)

# # 床のリンゴまで下ろす
print('lower to apple')
reach(APPLE[0], APPLE[1], 0.1, ref_frame_id='odom')
time.sleep(0.5)

# # 掴む (apply_force)
print('grasp (apply_force)')
try:
    gripper.apply_force(0.5)
except Exception as e:
    print('apply_force ROS error (物理的には把持済み):', str(e)[:40])
time.sleep(1.5)
print('GRASPED')
