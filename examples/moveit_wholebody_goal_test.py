#!/usr/bin/env python3
"""Send an RViz-equivalent whole-body pose goal through MoveGroup."""

import argparse
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def duration_seconds(duration):
    return duration.sec + duration.nanosec / 1_000_000_000.0


class MoveItWholeBodyGoalTest(Node):
    def __init__(self):
        super().__init__("moveit_wholebody_goal_test")
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def make_goal(self, args):
        goal = MoveGroup.Goal()
        request = goal.request
        request.workspace_parameters.header.frame_id = args.frame
        request.workspace_parameters.min_corner.x = -5.0
        request.workspace_parameters.min_corner.y = -5.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 5.0
        request.workspace_parameters.max_corner.y = 5.0
        request.workspace_parameters.max_corner.z = 3.0
        request.start_state.is_diff = True
        request.group_name = "whole_body"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = args.velocity_scale
        request.max_acceleration_scaling_factor = args.acceleration_scale

        constraints = Constraints()

        if args.joint_goal:
            for item in args.joint_goal:
                name, value = item.split("=", 1)
                joint = JointConstraint()
                joint.joint_name = name
                joint.position = float(value)
                joint.tolerance_above = args.joint_tolerance
                joint.tolerance_below = args.joint_tolerance
                joint.weight = 1.0
                constraints.joint_constraints.append(joint)
        else:
            position = PositionConstraint()
            position.header.frame_id = args.frame
            position.link_name = "hand_palm_link"
            region = SolidPrimitive()
            region.type = SolidPrimitive.SPHERE
            region.dimensions = [args.position_tolerance]
            position.constraint_region.primitives.append(region)
            target = Pose()
            target.position.x = args.x
            target.position.y = args.y
            target.position.z = args.z
            target.orientation.w = 1.0
            position.constraint_region.primitive_poses.append(target)
            position.weight = 1.0
            constraints.position_constraints.append(position)

            orientation = OrientationConstraint()
            orientation.header.frame_id = args.frame
            orientation.link_name = "hand_palm_link"
            orientation.orientation.x = args.qx
            orientation.orientation.y = args.qy
            orientation.orientation.z = args.qz
            orientation.orientation.w = args.qw
            orientation.absolute_x_axis_tolerance = args.orientation_tolerance
            orientation.absolute_y_axis_tolerance = args.orientation_tolerance
            orientation.absolute_z_axis_tolerance = args.orientation_tolerance
            orientation.weight = 1.0
            constraints.orientation_constraints.append(orientation)

        request.goal_constraints.append(constraints)
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = args.plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return goal

    def run(self, args):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/move_action is unavailable")

        start = time.monotonic()
        send_future = self.client.send_goal_async(self.make_goal(args))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=args.timeout)
        if not send_future.done():
            raise TimeoutError("timed out while sending MoveGroup goal")
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("MoveGroup rejected the goal")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=args.timeout)
        if not result_future.done():
            handle.cancel_goal_async()
            raise TimeoutError("timed out while waiting for MoveGroup result")

        wrapped = result_future.result()
        result = wrapped.result
        trajectory = result.planned_trajectory.joint_trajectory
        trajectory_duration = (
            duration_seconds(trajectory.points[-1].time_from_start)
            if trajectory.points
            else 0.0
        )
        print(f"action_status: {wrapped.status}", flush=True)
        print(f"moveit_error_code: {result.error_code.val}", flush=True)
        print(f"planning_time: {result.planning_time:.3f}s", flush=True)
        print(f"trajectory_points: {len(trajectory.points)}", flush=True)
        print(f"trajectory_duration: {trajectory_duration:.3f}s", flush=True)
        for index, name in enumerate(trajectory.joint_names):
            positions = [
                point.positions[index]
                for point in trajectory.points
                if index < len(point.positions)
            ]
            if positions:
                print(
                    f"{name}: min={min(positions):.4f} "
                    f"max={max(positions):.4f} final={positions[-1]:.4f}",
                    flush=True,
                )
        print(f"wall_seconds: {time.monotonic() - start:.3f}s", flush=True)
        if result.error_code.val != result.error_code.SUCCESS:
            raise RuntimeError(
                f"MoveGroup failed with error code {result.error_code.val}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=1.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.7)
    parser.add_argument("--qx", type=float, default=0.70710678)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.70710678)
    parser.add_argument("--qw", type=float, default=0.0)
    parser.add_argument("--frame", default="odom")
    parser.add_argument("--position-tolerance", type=float, default=0.01)
    parser.add_argument("--orientation-tolerance", type=float, default=0.03)
    parser.add_argument(
        "--joint-goal",
        action="append",
        metavar="NAME=POSITION",
        help="Use repeatable joint constraints instead of a hand pose goal",
    )
    parser.add_argument("--joint-tolerance", type=float, default=0.005)
    parser.add_argument("--velocity-scale", type=float, default=1.0)
    parser.add_argument("--acceleration-scale", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = MoveItWholeBodyGoalTest()
    try:
        node.run(args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
