#!/usr/bin/env python3
"""cmd_vel で台車を odom 目標へ位置決めする簡易ドライバ。

使い方: python3 drive_to.py <x> <y> <yaw>
回転→前進(方位補正つき)→最終回転 の3段階。TF (odom) をフィードバックに使う。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DriveTo(Node):
    def __init__(self, gx, gy, gyaw):
        super().__init__('drive_to')
        self.goal = (gx, gy, gyaw)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(
            Twist, '/omni_base_controller/cmd_vel', 10)
        self.phase = 0  # 0=目標方向へ回頭, 1=前進, 2=最終向き, 3=完了
        self.timer = self.create_timer(0.1, self.tick)

    def pose(self):
        t = self.buf.lookup_transform('odom', 'base_footprint',
                                      rclpy.time.Time())
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return tr.x, tr.y, yaw

    def tick(self):
        try:
            x, y, yaw = self.pose()
        except Exception:
            return
        gx, gy, gyaw = self.goal
        dist = math.hypot(gx - x, gy - y)
        cmd = Twist()
        if self.phase == 0:
            if dist < 0.03:
                self.phase = 2
                return
            aim = math.atan2(gy - y, gx - x)
            e = wrap(aim - yaw)
            if abs(e) < 0.05:
                self.phase = 1
            else:
                cmd.angular.z = max(-0.6, min(0.6, 1.2 * e))
        elif self.phase == 1:
            aim = math.atan2(gy - y, gx - x)
            e = wrap(aim - yaw)
            if dist < 0.02:
                self.phase = 2
            elif abs(e) > 0.4:
                self.phase = 0
            else:
                cmd.linear.x = max(0.05, min(0.25, 0.8 * dist))
                cmd.angular.z = max(-0.4, min(0.4, 1.0 * e))
        elif self.phase == 2:
            e = wrap(gyaw - yaw)
            if abs(e) < 0.02:
                self.phase = 3
                print(f'DONE pose=({x:+.3f},{y:+.3f},{yaw:+.3f})', flush=True)
            else:
                cmd.angular.z = max(-0.6, min(0.6, 1.2 * e))
        self.pub.publish(cmd)


def main():
    gx, gy, gyaw = (float(a) for a in sys.argv[1:4])
    rclpy.init()
    n = DriveTo(gx, gy, gyaw)
    t0 = time.time()
    while rclpy.ok() and n.phase != 3 and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.phase != 3:
        print('TIMEOUT', flush=True)
    # 停止を明示
    n.pub.publish(Twist())


if __name__ == '__main__':
    main()
