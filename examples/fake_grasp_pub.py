#!/usr/bin/env python3
"""シミュレータ真値から偽の GraspNet 検出結果を流す検証用パブリッシャー。

hsrb_pick_and_place はメッセージの header.frame_id を基準に座標変換するので、
odom フレームで「リンゴの真上から掴む姿勢」を流し続ければ、
認識パイプライン (YOLOX + GraspNet) 無しで pick & place の動作列を検証できる。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from graspnet_ros_msgs.msg import GraspDetection

GRASP_POS = (0.70, 0.0, 0.055)   # リンゴ odom(0.7,0) = 正面0.7m
# 真上から掴む姿勢: 手の z 軸 (掌の向き) を真下に = x 軸まわり 180 度回転
GRASP_QUAT = (1.0, 0.0, 0.0, 0.0)  # (x, y, z, w)


class FakeGraspPub(Node):
    def __init__(self):
        super().__init__('fake_grasp_pub')
        self.pub = self.create_publisher(
            GraspDetection, '/grasp_detector_node/result',
            qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.tick)  # 5Hz

    def tick(self):
        msg = GraspDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.is_not_framed_out = True
        from geometry_msgs.msg import Pose
        p = Pose()
        p.position.x, p.position.y, p.position.z = GRASP_POS
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = GRASP_QUAT
        msg.pose = [p]
        msg.width = [0.07]
        msg.height = [0.02]
        msg.depth = [0.02]
        msg.score = [1.0]
        msg.robustness = [3.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeGraspPub()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
