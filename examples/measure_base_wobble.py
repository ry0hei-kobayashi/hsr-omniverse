#!/usr/bin/env python3
"""Run one omni-base trajectory and print compact motion diagnostics as JSON."""

import argparse
import json
import math
import statistics
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from trajectory_msgs.msg import JointTrajectoryPoint


BASE_JOINTS = (
    "base_roll_joint",
    "base_l_drive_wheel_joint",
    "base_r_drive_wheel_joint",
)


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _stats(values):
    if not values:
        return {"count": 0}
    mean = statistics.fmean(values)
    centered = [value - mean for value in values]
    absolute_centered = [abs(value) for value in centered]
    return {
        "count": len(values),
        "mean": mean,
        "stddev": math.sqrt(statistics.fmean(
            value * value for value in centered)),
        "max_deviation": max(absolute_centered),
        "p95_deviation": _percentile(absolute_centered, 0.95),
        "p99_deviation": _percentile(absolute_centered, 0.99),
        "min": min(values),
        "max": max(values),
    }


class BaseWobbleMeasurement(Node):
    def __init__(self):
        super().__init__("measure_base_wobble")
        self.linear = [[], [], []]
        self.angular = [[], [], []]
        self.joint_velocity = {name: [] for name in BASE_JOINTS}
        self.last_joint_position = {}
        self.last_odom = None
        self.last_state = None
        self.create_subscription(
            Imu, "/hsrb/base_imu/data", self._on_imu,
            qos_profile_sensor_data)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state,
            qos_profile_sensor_data)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/omni_base_controller/state",
            self._on_controller_state,
            10,
        )
        self.create_subscription(
            Odometry,
            "/omni_base_controller/wheel_odom",
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/omni_base_controller/follow_joint_trajectory",
        )

    def _on_imu(self, msg):
        for index, value in enumerate((
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )):
            self.linear[index].append(float(value))
        for index, value in enumerate((
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )):
            self.angular[index].append(float(value))

    def _on_joint_state(self, msg):
        index_by_name = {name: index for index, name in enumerate(msg.name)}
        for name in BASE_JOINTS:
            index = index_by_name.get(name)
            if index is not None and index < len(msg.position):
                self.last_joint_position[name] = float(msg.position[index])
            if index is not None and index < len(msg.velocity):
                self.joint_velocity[name].append(float(msg.velocity[index]))

    def _on_controller_state(self, msg):
        self.last_state = msg

    def _on_odom(self, msg):
        self.last_odom = msg

    def reset_samples(self):
        self.linear = [[], [], []]
        self.angular = [[], [], []]
        self.joint_velocity = {name: [] for name in BASE_JOINTS}

    def make_goal(self, x, y, yaw, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["odom_x", "odom_y", "odom_t"]
        point = JointTrajectoryPoint()
        point.positions = [x, y, yaw]
        point.velocities = [0.0, 0.0, 0.0]
        whole_seconds = int(duration)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            (duration - whole_seconds) * 1_000_000_000)
        goal.trajectory.points = [point]
        return goal

    def report(self, action_status, action_error_code):
        axes = ("x", "y", "z")
        report = {
            "action_status": action_status,
            "action_error_code": action_error_code,
            "linear_acceleration": {
                axis: _stats(self.linear[index])
                for index, axis in enumerate(axes)
            },
            "angular_velocity": {
                axis: _stats(self.angular[index])
                for index, axis in enumerate(axes)
            },
            "joint_velocity": {
                name: _stats(values)
                for name, values in self.joint_velocity.items()
            },
            "final_joint_position": dict(self.last_joint_position),
        }
        if self.last_odom is not None:
            pose = self.last_odom.pose.pose
            q = pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            report["final_odom"] = {
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": yaw,
            }
        if self.last_state is not None:
            report["controller_state"] = {
                "joint_names": list(self.last_state.joint_names),
                "actual_positions": list(self.last_state.actual.positions),
                "desired_positions": list(self.last_state.desired.positions),
                "error_positions": list(self.last_state.error.positions),
            }
        return report


def _spin_for(node, duration):
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.3)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--move-duration", type=float, default=5.0)
    parser.add_argument("--baseline-duration", type=float, default=1.0)
    parser.add_argument("--post-duration", type=float, default=2.0)
    parser.add_argument("--wall-timeout", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = BaseWobbleMeasurement()
    action_status = None
    action_error_code = None
    try:
        if not node.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("omni-base action server is unavailable")
        _spin_for(node, args.baseline_duration)
        node.reset_samples()

        send_future = node.action_client.send_goal_async(
            node.make_goal(args.x, args.y, args.yaw, args.move_duration))
        deadline = time.monotonic() + args.wall_timeout
        while rclpy.ok() and not send_future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("timed out while sending the base goal")
            rclpy.spin_once(node, timeout_sec=0.05)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("omni-base goal was rejected")

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if time.monotonic() >= deadline:
                goal_handle.cancel_goal_async()
                raise TimeoutError("timed out while executing the base goal")
            rclpy.spin_once(node, timeout_sec=0.05)
        wrapped_result = result_future.result()
        action_status = int(wrapped_result.status)
        action_error_code = int(wrapped_result.result.error_code)
        _spin_for(node, args.post_duration)
        print(json.dumps(
            node.report(action_status, action_error_code),
            indent=2,
            sort_keys=True,
        ))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
