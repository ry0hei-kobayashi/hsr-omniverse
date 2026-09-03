#!/usr/bin/env python3
"""Measure arm-joint and IMU motion without commanding the robot."""

import argparse
import json
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState


ARM_JOINTS = (
    "arm_lift_joint",
    "arm_flex_joint",
    "arm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)
BASE_JOINTS = ("odom_x", "odom_y", "odom_t")


def _stats(values):
    if not values:
        return {"count": 0}
    mean = statistics.fmean(values)
    return {
        "count": len(values),
        "mean": mean,
        "stddev": math.sqrt(statistics.fmean(
            (value - mean) ** 2 for value in values)),
        "rms": math.sqrt(statistics.fmean(
            value * value for value in values)),
        "max_abs": max(abs(value) for value in values),
        "min": min(values),
        "max": max(values),
    }


class StabilityMeasurement(Node):
    def __init__(self):
        super().__init__("measure_joint_stability")
        self.position = {name: [] for name in ARM_JOINTS}
        self.velocity = {name: [] for name in ARM_JOINTS}
        self.base_position = {name: [] for name in BASE_JOINTS}
        self.base_velocity = {name: [] for name in BASE_JOINTS}
        self.odom_position = {axis: [] for axis in "xyt"}
        self.odom_velocity = {axis: [] for axis in "xyt"}
        self.linear_acceleration = {axis: [] for axis in "xyz"}
        self.angular_velocity = {axis: [] for axis in "xyz"}
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            "/whole_body_moveit/joint_states",
            self._on_base_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/hsrb/base_imu/data",
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/omni_base_controller/wheel_odom",
            self._on_odom,
            qos_profile_sensor_data,
        )

    def _on_joint_state(self, msg):
        index_by_name = {name: index for index, name in enumerate(msg.name)}
        for name in ARM_JOINTS:
            index = index_by_name.get(name)
            if index is None:
                continue
            if index < len(msg.position):
                self.position[name].append(float(msg.position[index]))
            if index < len(msg.velocity):
                self.velocity[name].append(float(msg.velocity[index]))

    def _on_base_joint_state(self, msg):
        index_by_name = {name: index for index, name in enumerate(msg.name)}
        for name in BASE_JOINTS:
            index = index_by_name.get(name)
            if index is None:
                continue
            if index < len(msg.position):
                self.base_position[name].append(float(msg.position[index]))
            if index < len(msg.velocity):
                self.base_velocity[name].append(float(msg.velocity[index]))

    def _on_imu(self, msg):
        for axis, value in zip("xyz", (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )):
            self.linear_acceleration[axis].append(float(value))
        for axis, value in zip("xyz", (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )):
            self.angular_velocity[axis].append(float(value))

    def _on_odom(self, msg):
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.odom_position["x"].append(float(msg.pose.pose.position.x))
        self.odom_position["y"].append(float(msg.pose.pose.position.y))
        self.odom_position["t"].append(yaw)
        self.odom_velocity["x"].append(float(msg.twist.twist.linear.x))
        self.odom_velocity["y"].append(float(msg.twist.twist.linear.y))
        self.odom_velocity["t"].append(float(msg.twist.twist.angular.z))

    def report(self):
        return {
            "joint_position": {
                name: _stats(values)
                for name, values in self.position.items()
            },
            "joint_velocity": {
                name: _stats(values)
                for name, values in self.velocity.items()
            },
            "base_position": {
                name: _stats(values)
                for name, values in self.base_position.items()
            },
            "base_velocity": {
                name: _stats(values)
                for name, values in self.base_velocity.items()
            },
            "odom_position": {
                axis: _stats(values)
                for axis, values in self.odom_position.items()
            },
            "odom_velocity": {
                axis: _stats(values)
                for axis, values in self.odom_velocity.items()
            },
            "linear_acceleration": {
                axis: _stats(values)
                for axis, values in self.linear_acceleration.items()
            },
            "angular_velocity": {
                axis: _stats(values)
                for axis, values in self.angular_velocity.items()
            },
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = StabilityMeasurement()
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        print(json.dumps(node.report(), indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
