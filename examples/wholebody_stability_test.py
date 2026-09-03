#!/usr/bin/env python3
"""Plan and optionally execute a repeatable whole-body stability motion."""

import argparse
import math
import time

from hsrb_interface import Robot, geometry


def duration_seconds(duration):
    return duration.sec + duration.nanosec / 1_000_000_000.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.9)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.65)
    parser.add_argument("--ei", type=float, default=0.0)
    parser.add_argument("--ej", type=float, default=math.pi / 2.0)
    parser.add_argument("--ek", type=float, default=0.0)
    parser.add_argument("--frame", default="odom")
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()

    robot = Robot()
    whole_body = robot.get("whole_body")
    goal = geometry.pose(
        x=args.x,
        y=args.y,
        z=args.z,
        ei=args.ei,
        ej=args.ej,
        ek=args.ek,
    )

    trajectory = whole_body.move_end_effector_pose(
        goal,
        ref_frame_id=args.frame,
        plan_only=True,
    )
    if trajectory is None or not trajectory.points:
        raise RuntimeError("MoveIt returned an empty whole-body trajectory")

    print("joints:", list(trajectory.joint_names), flush=True)
    print("points:", len(trajectory.points), flush=True)
    print(
        "duration:",
        f"{duration_seconds(trajectory.points[-1].time_from_start):.3f}s",
        flush=True,
    )
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

    if args.plan_only:
        return

    print("execute", flush=True)
    whole_body._execute_trajectory(trajectory)
    time.sleep(3.0)
    print("WHOLE_BODY_STABILITY_TEST_COMPLETED", flush=True)


if __name__ == "__main__":
    main()
