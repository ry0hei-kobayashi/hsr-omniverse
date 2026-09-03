#!/usr/bin/env python3
"""GraspDetection の pose を RViz 用 TF として配信する。

入力:  /grasp_detector_node/result (graspnet_ros_msgs/GraspDetection)
出力:  TF child frame grasp_point, grasp_point_1, ...
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from graspnet_ros_msgs.msg import GraspDetection
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class GraspTfBroadcaster(Node):
    def __init__(self):
        super().__init__('grasp_tf_broadcaster')
        self.declare_parameter('topic', '/grasp_detector_node/result')
        self.declare_parameter('child_prefix', 'grasp_point')
        self.declare_parameter('max_poses', 5)
        self.topic = self.get_parameter('topic').value
        self.child_prefix = self.get_parameter('child_prefix').value
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.last_msg = None
        self.br = TransformBroadcaster(self)
        self.create_subscription(
            GraspDetection,
            self.topic,
            self.on_grasp,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.publish_tf)
        self.get_logger().info(
            f'broadcasting grasp poses from {self.topic} as TF "{self.child_prefix}"')

    def on_grasp(self, msg):
        if msg.pose:
            self.last_msg = msg

    def publish_tf(self):
        if self.last_msg is None:
            return
        stamp = self.get_clock().now().to_msg()
        transforms = []
        parent = self.last_msg.header.frame_id or 'odom'
        for i, pose in enumerate(self.last_msg.pose[:self.max_poses]):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = (
                self.child_prefix if i == 0 else f'{self.child_prefix}_{i}')
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            transforms.append(t)
        self.br.sendTransform(transforms)


def main():
    rclpy.init()
    node = GraspTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
