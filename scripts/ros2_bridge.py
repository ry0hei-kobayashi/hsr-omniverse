# this code has been adapted from
#  https://github.com/Toni-SM/semu.robotics.ros2_bridge/blob/main/src/semu.robotics.ros2_bridge/semu/robotics/ros2_bridge/ros2_bridge.py
# the code is licensed under the MIT license
#  https://github.com/Toni-SM/semu.robotics.ros2_bridge/blob/main/LICENSE

from typing import List, Any

import math
import time
import json
import asyncio
import threading

import omni
import carb
import omni.kit
from pxr import Usd, Gf, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import get_stage_units

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from control_msgs.msg import JointTrajectoryControllerState


class RosController:
    def __init__(self, node: Node) -> None:
        """Base class for RosController

        :param node: ROS2 node
        :type node: Node
        """
        self._node = node        
        self.started = False
    
    def start(self) -> None:
        """Start the component
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Stop the component
        """
        print("[Info][semu.robotics.ros2_bridge] RosController: stopping")
        self.started = False

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        raise NotImplementedError


class RosControlFollowJointTrajectory(RosController):
    # Joints this controller manages. None = publish all articulation joints
    # (legacy behavior). Subclasses override with a list to scope the
    # /controller_state msg to the joints the consumer expects (pumas
    # arm_node segfaults when actual.positions oversized).
    controlled_joints = None
    goal_position_tolerance = None
    goal_velocity_tolerance = None
    goal_settle_duration = 0.0
    goal_timeout = 0.0

    def __init__(self,
                 node: Node,
                 _dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """FollowJointTrajectory interface
        
        :param node: The ROS node
        :type node: rclpy.node.Node
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node)

        self.dci = _dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_dt = 0.05
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_point_index = 1
        self._goal_wait_start_time = None
        self._goal_settle_start_time = None
        self._joint_velocity_samples = {}
        self._joint_velocity_filter_tau = 0.05

        # feedback / result
        self._action_result_message = None
        self._action_feedback_message = FollowJointTrajectory.Feedback()

        # publishers for controller state: legacy /<c>/state and the
        # ros2_control standard /<c>/controller_state (pumas's arm_controller
        # / viewpoint_controller subscribe to the latter)
        self._state_pub = None
        self._controller_state_pub = None

        # topic-interface subscription for /<c>/joint_trajectory (mirrors
        # ros2_control's JointTrajectoryController)
        self._traj_sub = None

        # Topic msgs are received on the ROS executor thread; the actual goal
        # installation (which calls DCI for current joint state) is deferred
        # to step() on the main thread to avoid racing with the physics step.
        self._pending_topic_msg = None
        self._pending_lock = threading.Lock()

        # Pad `time_from_start == 0` to this many seconds so single-shot
        # trajectories (e.g. pumas initial-pose publish) have time to
        # physically reach the target before the trajectory ends.
        self._zero_time_pad_sec = 1.0

    def start(self, _articulation_path, _action_topic_name) -> None:
        """Start the action server
        """
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: starting")

        # get attributes and relationships
        self.articulation_path = _articulation_path
        self.action_topic_name = _action_topic_name

        # start action server
        self._action_server = ActionServer(self._node,
                                           FollowJointTrajectory,
                                           self.action_topic_name,
                                           execute_callback=self._on_execute,
                                           goal_callback=self._on_goal,
                                           cancel_callback=self._on_cancel,
                                           handle_accepted_callback=self._on_handle_accepted)
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: register action {}" \
            .format(self.action_topic_name))

        # Controller state publishers — both topic names so topic-only clients
        # (pumas) and legacy consumers see the same payload.
        state_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/state')
        self._state_pub = self._node.create_publisher(
            JointTrajectoryControllerState,
            state_topic,
            10,
        )
        controller_state_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/controller_state')
        self._controller_state_pub = self._node.create_publisher(
            JointTrajectoryControllerState,
            controller_state_topic,
            10,
        )

        # Topic interface that mirrors ros2_control's JointTrajectoryController
        traj_topic = self.action_topic_name.replace('/follow_joint_trajectory', '/joint_trajectory')
        self._traj_sub = self._node.create_subscription(
            JointTrajectory,
            traj_topic,
            self._on_trajectory_topic,
            10,
        )
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: subscribe topic {}" \
            .format(traj_topic))
        print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: publish state on {} and {}" \
            .format(state_topic, controller_state_topic))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        # destroy action server
        if self._action_server is not None:
            print("[Info][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: destroy action server: {}" \
                .format(self.action_topic_name))
            # self._action_server.destroy()
            self._action_server = None
        # destroy topic subscription
        if self._traj_sub is not None:
            try:
                self._node.destroy_subscription(self._traj_sub)
            except Exception:
                pass
            self._traj_sub = None
        self._action_goal_handle = None
        self._action_goal = None

    def _duration_to_seconds(self, duration: Duration) -> float:
        """Convert a ROS2 Duration to seconds

        :param duration: The ROS2 Duration
        :type duration: Duration

        :return: The duration in seconds
        :rtype: float
        """
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        self._articulation = self.dci.get_articulation(self.articulation_path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: {} is not an articulation".format(path))
            return

        dof_props = self.dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self.dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self.dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self.dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self.dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                              "type": self.dci.get_joint_type(_joint),
                                              "dof": self.dci.find_articulation_dof(self._articulation, dof_name),
                                              "lower": lower_limits[i],
                                              "upper": upper_limits[i],
                                              "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: no joints found in {}".format(self.articulation_path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # Planner/filter bugs must never be allowed to inject NaN into PhysX.
        # One non-finite DOF target invalidates the entire articulation and all
        # subsequent TF, so skip it at the final boundary as a last line of
        # defense even though goals are validated on receipt below.
        if not math.isfinite(float(target_position)):
            key = (name, repr(target_position))
            warned = getattr(self, '_invalid_target_warned', set())
            if key not in warned:
                warned.add(key)
                self._invalid_target_warned = warned
                print(
                    '[trajectory-safety] {} ignored non-finite target: '
                    '{}={}'.format(
                        self.action_topic_name, name, target_position),
                    flush=True,
                )
            return
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self.dci.set_dof_position_target(self._joints[name]["dof"], target_position)
        self.dci.set_dof_velocity_target(self._joints[name]["dof"], 0.0)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self.dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _get_joint_velocity(self, name: str) -> float:
        # Isaac Sim 4.5 dynamic-control can report a constant non-zero DOF
        # velocity while the corresponding position is motionless (observed
        # on HSR's inverted arm joints).  Derive velocity from position and
        # simulation time so action tolerances describe visible motion.
        now = self._node.get_clock().now().nanoseconds / 1e9
        position = self._get_joint_position(name)
        previous = self._joint_velocity_samples.get(name)
        if previous is None:
            self._joint_velocity_samples[name] = (now, position, 0.0)
            return 0.0
        previous_time, previous_position, previous_velocity = previous
        dt = now - previous_time
        if dt <= 1e-9:
            return previous_velocity
        position_delta = position - previous_position
        if self._joints[name]["type"] == _dynamic_control.JOINT_REVOLUTE:
            position_delta = math.atan2(
                math.sin(position_delta),
                math.cos(position_delta),
            )
        raw_velocity = position_delta / dt
        alpha = min(
            1.0,
            dt / (self._joint_velocity_filter_tau + dt),
        )
        velocity = previous_velocity + alpha * (
            raw_velocity - previous_velocity)
        self._joint_velocity_samples[name] = (now, position, velocity)
        return velocity

    @staticmethod
    def _joint_tolerance(tolerance, name):
        if isinstance(tolerance, dict):
            return tolerance.get(name)
        return tolerance

    def _check_goal_convergence(self):
        if (
            self._action_goal_handle is None
            or self.goal_position_tolerance is None
            or self.goal_velocity_tolerance is None
        ):
            return "succeeded", ""

        now = self._node.get_clock().now().nanoseconds / 1e9
        if self._goal_wait_start_time is None:
            self._goal_wait_start_time = now

        final_point = self._action_goal.trajectory.points[-1]
        position_errors = {}
        velocities = {}
        settled = True
        for index, name in enumerate(
            self._action_goal.trajectory.joint_names
        ):
            if index >= len(final_point.positions):
                continue
            position_error = (
                final_point.positions[index]
                - self._get_joint_position(name)
            )
            velocity = self._get_joint_velocity(name)
            position_errors[name] = position_error
            velocities[name] = velocity
            position_tolerance = self._joint_tolerance(
                self.goal_position_tolerance, name)
            velocity_tolerance = self._joint_tolerance(
                self.goal_velocity_tolerance, name)
            if (
                position_tolerance is None
                or velocity_tolerance is None
                or abs(position_error) > position_tolerance
                or abs(velocity) > velocity_tolerance
            ):
                settled = False

        if settled:
            if self._goal_settle_start_time is None:
                self._goal_settle_start_time = now
            if now - self._goal_settle_start_time >= self.goal_settle_duration:
                return "succeeded", ""
        else:
            self._goal_settle_start_time = None

        if now - self._goal_wait_start_time < self.goal_timeout:
            return "waiting", ""

        worst_position = max(
            position_errors.items(), key=lambda item: abs(item[1]))
        worst_velocity = max(
            velocities.items(), key=lambda item: abs(item[1]))
        detail = (
            "goal did not settle: position %s=%+.4f, velocity %s=%+.4f"
            % (
                worst_position[0],
                worst_position[1],
                worst_velocity[0],
                worst_velocity[1],
            )
        )
        return "aborted", detail

    def _reset_goal_convergence(self) -> None:
        self._goal_wait_start_time = None
        self._goal_settle_start_time = None

    def _on_handle_accepted(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> None:
        """Callback function for handling newly accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle
        """
        goal_handle.execute()

    def _on_goal(self, goal: 'FollowJointTrajectory.Goal') -> 'rclpy.action.server.GoalResponse':
        """Callback function for handling new goal requests

        :param goal: The goal
        :type goal: FollowJointTrajectory.Goal

        :return: Whether the goal was accepted
        :rtype: rclpy.action.server.GoalResponse
        """
        # reject if joints don't match
        for name in goal.trajectory.joint_names:
            if name not in self._joints:
                print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                return GoalResponse.REJECT

        joint_count = len(goal.trajectory.joint_names)
        if not goal.trajectory.points:
            print(
                '[trajectory-safety] {} rejected empty trajectory'.format(
                    self.action_topic_name),
                flush=True,
            )
            return GoalResponse.REJECT
        for point_index, point in enumerate(goal.trajectory.points):
            if len(point.positions) < joint_count:
                print(
                    '[trajectory-safety] {} rejected short positions at point '
                    '{}: {} < {}'.format(
                        self.action_topic_name, point_index,
                        len(point.positions), joint_count),
                    flush=True,
                )
                return GoalResponse.REJECT
            invalid_positions = [
                (goal.trajectory.joint_names[i], point.positions[i])
                for i in range(joint_count)
                if not math.isfinite(float(point.positions[i]))
            ]
            if invalid_positions:
                print(
                    '[trajectory-safety] {} rejected non-finite positions at '
                    'point {}: {}'.format(
                        self.action_topic_name, point_index,
                        invalid_positions),
                    flush=True,
                )
                return GoalResponse.REJECT
            if point.velocities:
                sanitized = list(point.velocities)
                changed = []
                for i, value in enumerate(sanitized):
                    if not math.isfinite(float(value)):
                        changed.append((i, value))
                        sanitized[i] = 0.0
                if changed:
                    point.velocities = sanitized
                    print(
                        '[trajectory-safety] {} replaced non-finite velocities '
                        'with zero at point {}: {}'.format(
                            self.action_topic_name, point_index, changed),
                        flush=True,
                    )

        # Preempt any in-flight goal so streaming clients can replace the
        # active trajectory. The previous _on_execute loop polls _action_goal
        # and exits with INVALID_GOAL once we clear it.
        if self._action_goal is not None:
            old_handle = self._action_goal_handle
            self._action_goal = None
            if old_handle is not None and old_handle.is_active:
                try:
                    old_handle.abort()
                except Exception as exc:
                    print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: abort failed: {}" \
                        .format(exc))
            self._action_goal_handle = None

        # Pad zero time_from_start so single-point trajectories become a
        # 2-point interpolation with enough physical time to converge.
        if goal.trajectory.points:
            first_tfs = goal.trajectory.points[0].time_from_start
            if first_tfs.sec == 0 and first_tfs.nanosec == 0:
                pad_total_ns = int(self._zero_time_pad_sec * 1e9)
                goal.trajectory.points[0].time_from_start.sec = pad_total_ns // 1_000_000_000
                goal.trajectory.points[0].time_from_start.nanosec = pad_total_ns % 1_000_000_000

        # check initial position
        if goal.trajectory.points[0].time_from_start:
            initial_point = JointTrajectoryPoint(
                positions=[
                    self._get_joint_position(name)
                    for name in goal.trajectory.joint_names
                ],
                velocities=[0.0] * len(goal.trajectory.joint_names),
                time_from_start=Duration().to_msg(),
            )
            goal.trajectory.points.insert(0, initial_point)

        # reset internal data
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_point_index = 1
        self._reset_goal_convergence()

        # store goal data
        self._action_goal = goal

        return GoalResponse.ACCEPT

    def _on_trajectory_topic(self, msg: JointTrajectory) -> None:
        """Topic-interface entry point.

        Runs on the ROS executor thread. To avoid racing with the physics
        simulation on DCI calls, this method only validates the message and
        stashes it. step() picks it up on the main thread.
        """
        for name in msg.joint_names:
            if name not in self._joints:
                print("[Warning][semu.robotics.ros2_bridge] RosControlFollowJointTrajectory: topic msg joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                return
        if not msg.points:
            return
        joint_count = len(msg.joint_names)
        for point_index, point in enumerate(msg.points):
            if len(point.positions) < joint_count or any(
                not math.isfinite(float(value))
                for value in point.positions[:joint_count]
            ):
                print(
                    '[trajectory-safety] {} ignored invalid topic trajectory '
                    'point {}'.format(self.action_topic_name, point_index),
                    flush=True,
                )
                return
            if point.velocities:
                point.velocities = [
                    float(value) if math.isfinite(float(value)) else 0.0
                    for value in point.velocities
                ]
        with self._pending_lock:
            self._pending_topic_msg = msg

    def _install_pending_trajectory(self) -> None:
        """Promote a pending topic msg to the active trajectory (main thread only)."""
        with self._pending_lock:
            msg = self._pending_topic_msg
            self._pending_topic_msg = None
        if msg is None:
            return

        # Preempt any in-flight trajectory.
        if self._action_goal is not None:
            self._action_goal = None
            self._action_goal_handle = None

        # Single-point trajectories are setpoint commands ("go to this pose").
        # Bypass the interpolator entirely and write joint targets directly —
        # the DCI position controller handles physical convergence. Running a
        # state-machine interpolation per msg means each new streaming goal
        # preempts the previous one mid-flight (pumas head_node at 10 Hz with
        # 1 s pad → ~10% progress per cycle; pumas arm_node with 0.2 s →
        # ~50% per cycle), which makes the joint barely track the stream.
        # time_from_start on a single point is only a deadline hint, not a
        # forced interpolation duration.  The HSR owner wakes its articulation
        # once per simulation step; waking this controller's independently
        # cached handle is both redundant and invalid for virtual odom joints.
        if len(msg.points) == 1:
            point = msg.points[0]
            for i, name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    self._set_joint_position(name, point.positions[i])
            return

        # Multi-point / non-zero trajectory: run the interpolator.
        # Insert current state as t=0 anchor for interpolation.
        initial_point = JointTrajectoryPoint(
            positions=[
                self._get_joint_position(name) for name in msg.joint_names
            ],
            velocities=[0.0] * len(msg.joint_names),
            time_from_start=Duration().to_msg(),
        )
        msg.points.insert(0, initial_point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        self._action_point_index = 1
        self._reset_goal_convergence()
        self._action_goal = goal
        # _action_goal_handle stays None -> step() treats this as topic mode

    def _on_cancel(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'rclpy.action.server.CancelResponse':
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: Whether the goal was canceled
        :rtype: rclpy.action.server.CancelResponse
        """
        if self._action_goal is None:
            return CancelResponse.REJECT
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._reset_goal_convergence()
        # ServerGoalHandle の破棄は rclpy ActionServer に任せる。
        # ここで手動 destroy すると、result_timeout 後の期限切れ処理が同じ
        # handle を再度破棄して KeyError となり、ROS executor 全体が停止する。
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'FollowJointTrajectory.Result':
        """Callback function for processing accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: The result of the goal execution
        :rtype: FollowJointTrajectory.Result
        """
        # reset internal data
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        # set goal
        self._action_goal_handle = goal_handle
        # wait for the goal to be executed
        while self._action_result_message is None: 
            if self._action_goal is None:
                result = FollowJointTrajectory.Result()
                result.error_code = result.INVALID_GOAL
                return result
            time.sleep(self._action_dt)
        self._action_goal = None
        self._action_goal_handle = None
        return self._action_result_message

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # Promote any pending topic-mode trajectory (received on the ROS
        # executor thread) so the DCI call for current joint state happens
        # here on the main thread.
        if self._pending_topic_msg is not None:
            self._install_pending_trajectory()
        # update articulation if a trajectory is active (action or topic mode)
        if self._action_goal is not None:
            self._action_dt = dt
            # end of trajectory
            if self._action_point_index >= len(self._action_goal.trajectory.points):
                convergence, detail = self._check_goal_convergence()
                if convergence != "waiting":
                    handle = self._action_goal_handle
                    self._action_goal = None
                    # 結果メッセージを先に公開すると _on_execute (executor
                    # スレッド) が即座に return し、rclpy 側が「終了状態が未設定」
                    # と判断して abort を呼ぶ。こちらの abort と競合して
                    # 「invalid transition」例外になり、ROS スレッドごと落ちて
                    # 全アクションが永久に無応答になる。必ず終了状態を先に
                    # 確定させてから結果を公開する。
                    result = FollowJointTrajectory.Result()
                    if convergence == "succeeded":
                        result.error_code = result.SUCCESSFUL
                        if handle is not None and handle.is_active:
                            try:
                                handle.succeed()
                            except Exception:
                                pass
                    else:
                        result.error_code = result.GOAL_TOLERANCE_VIOLATED
                        result.error_string = detail
                        print(
                            "[trajectory-action] " + detail,
                            flush=True,
                        )
                        if handle is not None and handle.is_active:
                            try:
                                handle.abort()
                            except Exception:
                                pass
                    self._action_result_message = result
                    self._action_goal_handle = None
                    self._reset_goal_convergence()
                    return
            else:
                previous_point = self._action_goal.trajectory.points[
                    self._action_point_index - 1]
                current_point = self._action_goal.trajectory.points[
                    self._action_point_index]
                if self._action_start_time is None:
                    # 開始時刻未設定のまま物理ステップが先行するレースのガード
                    self._action_start_time = (
                        self._node.get_clock().now().nanoseconds / 1e9)
                time_passed = (
                    self._node.get_clock().now().nanoseconds / 1e9
                    - self._action_start_time
                )

                # Interpolate the active segment.  MoveIt supplies endpoint
                # velocities, so use cubic Hermite interpolation when available
                # to keep target velocity continuous across trajectory points.
                if time_passed <= self._duration_to_seconds(
                    current_point.time_from_start
                ):
                    previous_time = self._duration_to_seconds(
                        previous_point.time_from_start)
                    current_time = self._duration_to_seconds(
                        current_point.time_from_start)
                    segment_duration = current_time - previous_time
                    ratio = 1.0 if segment_duration <= 0.0 else min(
                        1.0,
                        max(0.0, (time_passed - previous_time) /
                            segment_duration),
                    )
                    for i, name in enumerate(
                        self._action_goal.trajectory.joint_names
                    ):
                        previous_position = previous_point.positions[i]
                        current_position = current_point.positions[i]
                        if (
                            segment_duration > 0.0 and
                            i < len(previous_point.velocities) and
                            i < len(current_point.velocities)
                        ):
                            ratio2 = ratio * ratio
                            ratio3 = ratio2 * ratio
                            target_position = (
                                (2.0 * ratio3 - 3.0 * ratio2 + 1.0) *
                                previous_position
                                + (ratio3 - 2.0 * ratio2 + ratio) *
                                segment_duration *
                                previous_point.velocities[i]
                                + (-2.0 * ratio3 + 3.0 * ratio2) *
                                current_position
                                + (ratio3 - ratio2) * segment_duration *
                                current_point.velocities[i]
                            )
                        else:
                            target_position = previous_position + ratio * (
                                current_position - previous_position)
                        self._set_joint_position(name, target_position)
                else:
                    # Finish the current segment exactly.  The old code
                    # advanced the index first and wrote the next point for one
                    # frame, then interpolated back near the current point.
                    for i, name in enumerate(
                        self._action_goal.trajectory.joint_names
                    ):
                        if i < len(current_point.positions):
                            self._set_joint_position(
                                name, current_point.positions[i])
                    self._action_point_index += 1
                    if self._action_goal_handle is not None:
                        self._action_feedback_message.joint_names = list(
                            self._action_goal.trajectory.joint_names)
                        self._action_feedback_message.actual.positions = [
                            self._get_joint_position(name)
                            for name in
                            self._action_goal.trajectory.joint_names
                        ]
                        self._action_feedback_message.actual.time_from_start = (
                            Duration(seconds=time_passed).to_msg())
                        try:
                            self._action_goal_handle.publish_feedback(
                                self._action_feedback_message)
                        except Exception:
                            pass
        # publish controller state for this timestep
        if self._state_pub is not None:
            msg = JointTrajectoryControllerState()
            msg.header.stamp = self._node.get_clock().now().to_msg()

            # Scope joint_names to this controller's joints when declared;
            # otherwise fall back to all articulation joints. Skip names that
            # aren't registered in self._joints (defensive).
            if self.controlled_joints:
                joint_names = [n for n in self.controlled_joints if n in self._joints]
            else:
                joint_names = sorted(self._joints.keys())
            msg.joint_names = joint_names

            # actual state from articulation
            actual_positions = []
            actual_velocities = []
            for name in joint_names:
                try:
                    pos = self._get_joint_position(name)
                except Exception:
                    pos = 0.0
                actual_positions.append(pos)
                try:
                    vel = self._get_joint_velocity(name)
                except Exception:
                    vel = 0.0
                actual_velocities.append(vel)

            msg.actual.positions = list(actual_positions)
            msg.actual.velocities = list(actual_velocities)

            # desired state from current trajectory point if a goal is active
            desired_positions = list(actual_positions)
            desired_velocities = [0.0] * len(joint_names)

            if self._action_goal is not None and self._action_goal.trajectory.points:
                # clamp index in case it's already at/after the end
                point_index = min(self._action_point_index, len(self._action_goal.trajectory.points) - 1)
                current_point = self._action_goal.trajectory.points[point_index]

                # map trajectory joint order to controller joint order
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    if name in self._joints and name in joint_names:
                        j_idx = joint_names.index(name)
                        if i < len(current_point.positions):
                            desired_positions[j_idx] = current_point.positions[i]
                        if i < len(current_point.velocities):
                            desired_velocities[j_idx] = current_point.velocities[i]

            msg.desired.positions = desired_positions
            msg.desired.velocities = desired_velocities

            # error = desired - actual
            error_positions = [
                dp - ap for dp, ap in zip(desired_positions, actual_positions)
            ]
            error_velocities = [
                dv - av for dv, av in zip(desired_velocities, actual_velocities)
            ]
            msg.error.positions = error_positions
            msg.error.velocities = error_velocities

            # ROS 2 control_msgs/JointTrajectoryControllerState has BOTH the
            # legacy fields (actual/desired/error) and the new ones
            # (feedback/reference/error/output). pumas's arm_node reads
            # `feedback.positions[i]` without a size check and segfaults if
            # left empty — mirror the legacy data into the new fields.
            msg.feedback.positions = list(actual_positions)
            msg.feedback.velocities = list(actual_velocities)
            msg.reference.positions = list(desired_positions)
            msg.reference.velocities = list(desired_velocities)
            msg.output.positions = list(desired_positions)
            msg.output.velocities = list(desired_velocities)

            self._state_pub.publish(msg)
            if self._controller_state_pub is not None:
                self._controller_state_pub.publish(msg)

class RosControllerGripperCommand(RosController):
    def __init__(self,
                 node: Node,
                 _dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """GripperCommand interface

        :param node: The ROS node
        :type node: rclpy.node.Node
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node)
        
        self.dci = _dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_dt = 0.05
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        # TODO: add to schema?
        self._action_timeout = 10.0
        self._action_position_threshold = 0.001
        self._action_previous_position_sum = float("inf")

        # feedback / result
        self._action_result_message = None
        self._action_feedback_message = GripperCommand.Feedback()
        self._action_type = GripperCommand  # サブクラスが実機の型に上書き可能

    def start(self, _articulation_path, _action_topic_name) -> None:
        """Start the action server
        """
        print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: starting {}" \
            .format(_action_topic_name))

        # get attributes and relationships
        self.articulation_path = _articulation_path
        self.action_topic_name = _action_topic_name

        # start action server
        self._action_server = ActionServer(self._node,
                                           self._action_type,
                                           self.action_topic_name,
                                           execute_callback=self._on_execute,
                                           goal_callback=self._on_goal,
                                           cancel_callback=self._on_cancel,
                                           handle_accepted_callback=self._on_handle_accepted)
        print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: register action {}" \
            .format(self.action_topic_name))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        # destroy action server
        if self._action_server is not None:
            print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: destroy action server")
            # self._action_server.destroy()
            self._action_server = None
        self._action_goal_handle = None
        self._action_goal = None

    def _duration_to_seconds(self, duration: Duration) -> float:
        """Convert a ROS2 Duration to seconds

        :param duration: The ROS2 Duration
        :type duration: Duration

        :return: The duration in seconds
        :rtype: float
        """
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        self._articulation = self.dci.get_articulation(self.articulation_path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros2_bridge] RosControllerGripperCommand: {} is not an articulation".format(self.articulation_path))
            return

        dof_props = self.dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self.dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self.dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self.dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self.dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                                "type": self.dci.get_joint_type(_joint),
                                                "dof": self.dci.find_articulation_dof(self._articulation, dof_name),
                                                "lower": lower_limits[i],
                                                "upper": upper_limits[i],
                                                "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros2_bridge] RosControllerGripperCommand: no joints found in {}".format(path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self.dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self.dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_handle_accepted(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> None:
        """Callback function for handling newly accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle
        """
        goal_handle.execute()

    def _on_goal(self, goal: 'GripperCommand.Goal') -> 'rclpy.action.server.GoalResponse':
        """Callback function for handling new goal requests

        :param goal: The goal
        :type goal: GripperCommand.Goal

        :return: Whether the goal was accepted
        :rtype: rclpy.action.server.GoalResponse
        """
        # 以前のゴールが残っていても reject せず preempt (最新コマンド優先)
        if self._action_goal is not None:
            print("[Info][semu.robotics.ros2_bridge] RosControllerGripperCommand: preempting previous goal")

        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")

        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'rclpy.action.server.CancelResponse':
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: Whether the goal was canceled
        :rtype: rclpy.action.server.CancelResponse
        """
        if self._action_goal is None:
            return CancelResponse.REJECT
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")
        goal_handle.destroy()
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'GripperCommand.Result':
        """Callback function for processing accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: The result of the goal execution
        :rtype: GripperCommand.Result
        """
        # reset internal data
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")
        # set goal
        self._action_goal_handle = goal_handle
        self._action_goal = goal_handle.request
        # wait for the goal to be executed
        while self._action_result_message is None:
            if self._action_goal is None:
                # サーバのアクション型に合った Result を返す
                return self._action_type.Result()
            time.sleep(self._action_dt)
        self._action_goal = None
        self._action_goal_handle = None
        return self._action_result_message

    def step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        if self._action_goal is not None and self._action_goal_handle is not None:
            self._action_dt = dt
            target_position = self._action_goal.command.position
            # set target
            self.dci.wake_up_articulation(self._articulation)
            for name in self._joints:
                self._set_joint_position(name, target_position)
            # end (position reached)
            position = 0
            current_position_sum = 0
            position_reached = True
            for name in self._joints:
                position = self._get_joint_position(name)
                current_position_sum += position
                if abs(position - target_position) > self._action_position_threshold:
                    position_reached = False
                    break
            if position_reached:
                self._action_result_message = GripperCommand.Result()
                self._action_result_message.position = position
                self._action_result_message.stalled = False
                self._action_result_message.reached_goal = True
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                return
            # end (stalled)
            if abs(current_position_sum - self._action_previous_position_sum) < 1e-6:
                self._action_result_message = GripperCommand.Result()
                self._action_result_message.position = position
                self._action_result_message.stalled = True
                self._action_result_message.reached_goal = False
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                return
            self._action_previous_position_sum = current_position_sum
            # end (timeout)
            time_passed = self._node.get_clock().now().nanoseconds / 1e9 - self._action_start_time
            if time_passed >= self._action_timeout:
                self._action_result_message = GripperCommand.Result()
                if self._action_goal_handle is not None:
                    self._action_goal_handle.abort()
                    self._action_goal_handle = None
            # TODO: send feedback
            # self._action_goal_handle.publish_feedback(self._action_feedback_message)
