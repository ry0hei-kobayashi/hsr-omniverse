# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

import math
import os
import sys
import threading

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.ui
# from omni.isaac.core.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils import extensions, nucleus, stage, viewports
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils.render_product import create_hydra_texture
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

# from omni.isaac.sensor import _sensor

# add by r.kobayashi version selector
ROS_VERSION = os.environ.get('HSR_ROS_VERSION', '2').strip()
if ROS_VERSION not in ('1', '2'):
    raise ValueError(
        f"Unsupported HSR_ROS_VERSION={ROS_VERSION}. Use '1' or '2'.")
is_ros2 = ROS_VERSION == '2'

# is_ros2 = False
if is_ros2:
    import rclpy
    import rclpy.node
    import rclpy.qos
    import rclpy.time
    import tf_transformations
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from geometry_msgs.msg import (PoseStamped, Quaternion, TransformStamped,
                                   Twist, WrenchStamped)
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState
    from tf2_ros import TransformBroadcaster
    from tmc_control_msgs.action import GripperApplyEffort
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    def quaternion_from_euler(r, p, y):
        return tf_transformations.quaternion_from_euler(r, p, y)

    def euler_from_quaternion(x, y, z, w):
        return tf_transformations.euler_from_quaternion((x, y, z, w))

    extensions.enable_extension('isaacsim.ros2.bridge')

else:
    import actionlib
    import rospy
    import tf.transformations
    from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
    from control_msgs.msg import (FollowJointTrajectoryAction,
                                  FollowJointTrajectoryActionGoal,
                                  FollowJointTrajectoryGoal,
                                  GripperCommandAction,
                                  GripperCommandActionGoal)
    from geometry_msgs.msg import (PoseStamped, Quaternion, TransformStamped,
                                   Twist, WrenchStamped)
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState
    from tmc_control_msgs.msg import (GripperApplyEffortAction,
                                      GripperApplyEffortFeedback,
                                      GripperApplyEffortResult)
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    def quaternion_from_euler(r, p, y):
        return tf.transformations.quaternion_from_euler(r, p, y)

    def euler_from_quaternion(x, y, z, w):
        return tf.transformations.euler_from_quaternion((x, y, z, w))

    extensions.enable_extension('isaacsim.ros1.bridge')
# try:
#    import rospy
#    import tf.transformations
#    import actionlib
#    from geometry_msgs.msg import Twist, PoseStamped, Quaternion, WrenchStamped, TransformStamped
#    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandActionGoal
#    from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
#    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#    from sensor_msgs.msg import JointState, Imu
#    from nav_msgs.msg import Odometry
#    from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortResult, GripperApplyEffortFeedback
#
#    def quaternion_from_euler(r, p, y):
#        return tf.transformations.quaternion_from_euler(r, p, y)
#
#    def euler_from_quaternion(x, y, z, w):
#        return tf.transformations.euler_from_quaternion((x, y, z, w))
#
#    extensions.enable_extension("isaacsim.ros1.bridge")
# except ImportError:
#
#    is_ros2 = True
#    import rclpy
#    import rclpy.node
#    import rclpy.qos
#    import rclpy.time
#    from geometry_msgs.msg import Twist, PoseStamped, Quaternion, WrenchStamped, TransformStamped
#    from control_msgs.action import FollowJointTrajectory, GripperCommand
#    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#    from sensor_msgs.msg import JointState, Imu
#    from nav_msgs.msg import Odometry
#    from tf2_ros import TransformBroadcaster
#    from tmc_control_msgs.action import GripperApplyEffort
#    from rcl_interfaces.srv import GetParameters
#    from rcl_interfaces.msg import ParameterValue, ParameterType
#    import tf_transformations
#
#    def quaternion_from_euler(r, p, y):
#        return tf_transformations.quaternion_from_euler(r, p, y)
#
#    def euler_from_quaternion(x, y, z, w):
#        return tf_transformations.euler_from_quaternion((x, y, z, w))
#
#    extensions.enable_extension("isaacsim.ros2.bridge")

extensions.enable_extension('isaacsim.sensors.physx')
extensions.enable_extension('isaacsim.util.debug_draw')


def og_ros_node(name):
    if is_ros2:
        return name.replace('.ros1.bridge.', '.ros2.bridge.').replace('.ROS1', '.ROS2')
    return name


if is_ros2 is False:
    print('local module import ')
    sys.path.insert(0, os.path.join(os.path.dirname(
        os.path.abspath(__file__)), '..', '3rdparty'))
    from OgnROS1ActionFollowJointTrajectory import \
        InternalState as semuInternalState
    from OgnROS1ActionGripperCommand import \
        InternalState as semuGripperInternalState

    # extensions.enable_extension("semu.robotics.ros_bridge")
    # from semu.robotics.ros_bridge.ogn.nodes.OgnROS1ActionFollowJointTrajectory import InternalState as semuInternalState
    # from semu.robotics.ros_bridge.ogn.nodes.OgnROS1ActionGripperCommand import InternalState as semuGripperInternalState
else:  # ROS2
    from ros2_bridge import \
        RosControlFollowJointTrajectory as semuInternalState
    from ros2_bridge import \
        RosControllerGripperCommand as semuGripperInternalState


class odom_trajectory_action_server(semuInternalState):
    def _init_articulation(self) -> None:
        path = self.articulation_path
        self._articulation = self.dci.get_articulation(path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print(
                '[Warning] FollowJointTrajectory: {} is not an articulation'.format(path))
            return
        for dof_name in ['odom_x', 'odom_y', 'odom_t']:
            self._joints[dof_name] = 0.0
        self._odometry = None
        self._velocity = None
        self._remaining_start_time = None
        self._settle_start_time = None

    def _set_joint_position(self, name: str, target_position: float) -> None:
        if not math.isfinite(float(target_position)):
            print(
                '[trajectory-safety] ignored non-finite base target: '
                '%s=%r' % (name, target_position),
                flush=True,
            )
            return
        self._joints[name] = target_position

    def _get_joint_position(self, name: str) -> float:
        return self._joints[name]

    def _get_time(self) -> float:
        if is_ros2:
            return self._node.get_clock().now().nanoseconds / 1e9
        return rospy.get_time()

    def step(self, dt: float) -> None:
        if self._action_goal is not None and self._action_goal_handle is not None:
            if self._odometry is not None and self._action_point_index >= len(
                self._action_goal.trajectory.points
            ):
                errors = (
                    self._get_joint_position('odom_x') - self._odometry.x,
                    self._get_joint_position('odom_y') - self._odometry.y,
                    math.atan2(
                        math.sin(
                            self._get_joint_position('odom_t')
                            - self._odometry.ang
                        ),
                        math.cos(
                            self._get_joint_position('odom_t')
                            - self._odometry.ang
                        ),
                    ),
                )
                max_error = max(abs(error) for error in errors)
                velocities = self._velocity or (float('inf'),) * 3
                max_velocity = max(abs(velocity) for velocity in velocities)
                now = self._get_time()
                if self._remaining_start_time is None:
                    self._remaining_start_time = now
                time_passed = now - self._remaining_start_time
                converged = (
                    max_error <= _BASE_GOAL_TOLERANCE
                    and max_velocity <= _BASE_GOAL_VELOCITY_TOLERANCE
                )
                if converged:
                    if self._settle_start_time is None:
                        self._settle_start_time = now
                    if now - self._settle_start_time < _BASE_GOAL_SETTLE_DURATION:
                        return
                    print(
                        '[base-action] reached: '
                        'target=(%.4f, %.4f, %.4f) '
                        'actual=(%.4f, %.4f, %.4f) '
                        'error=(%.4f, %.4f, %.4f) '
                        'velocity=(%.4f, %.4f, %.4f)'
                        % ((
                                self._get_joint_position('odom_x'),
                                self._get_joint_position('odom_y'),
                                self._get_joint_position('odom_t'),
                                self._odometry.x,
                                self._odometry.y,
                                self._odometry.ang,
                            ) + errors + tuple(velocities)),
                        flush=True,
                    )
                else:
                    self._settle_start_time = None
                if not converged and time_passed < _BASE_GOAL_TIMEOUT:
                    return
                if not converged and is_ros2:
                    result = FollowJointTrajectory.Result()
                    result.error_code = result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = (
                        'base goal tolerance violated: '
                        'dx=%.4f dy=%.4f dyaw=%.4f '
                        'vx=%.4f vy=%.4f vyaw=%.4f'
                        % (errors + tuple(velocities))
                    )
                    print('[base-action] ' + result.error_string, flush=True)
                    handle = self._action_goal_handle
                    self._action_result_message = result
                    if handle is not None:
                        try:
                            handle.abort()
                        except Exception:
                            pass
                    self._action_goal_handle = None
                    self._action_goal = None
                    self._remaining_start_time = None
                    self._settle_start_time = None
                    return
            else:
                self._remaining_start_time = None
                self._settle_start_time = None
        super().step(dt)

    def sample_desired_velocity(self):
        """現在の軌道セグメントの「目標速度」(odom_x/y/t, odom系) を時刻補間で取り出す。

        本物の OmniBaseController と同じく、軌道の各点が持つ velocities を
        フィードフォワード(FF)として使うための источник。位置の差分から速度を
        自作するとセグメント境界でスパイク暴発するため、ここでは軌道が運んでいる
        速度をそのまま読む。velocities が無い軌道なら (0,0,0) を返す(=FFなし)。
        """
        g = self._action_goal
        if g is None:
            return (0.0, 0.0, 0.0)
        pts = g.trajectory.points
        idx = self._action_point_index
        if idx <= 0 or idx >= len(pts):
            return (0.0, 0.0, 0.0)
        prev_p = pts[idx - 1]
        cur_p = pts[idx]
        if not cur_p.velocities or not prev_p.velocities:
            return (0.0, 0.0, 0.0)
        try:
            t0 = self._duration_to_seconds(prev_p.time_from_start)
            t1 = self._duration_to_seconds(cur_p.time_from_start)
            tp = self._get_time() - (self._action_start_time
                                     if self._action_start_time is not None
                                     else self._get_time())
            ratio = 0.0 if t1 <= t0 else max(0.0, min(1.0, (tp - t0) / (t1 - t0)))
        except Exception:
            ratio = 0.0
        out = {'odom_x': 0.0, 'odom_y': 0.0, 'odom_t': 0.0}
        names = g.trajectory.joint_names
        for i, nm in enumerate(names):
            if nm in out and i < len(cur_p.velocities) and i < len(prev_p.velocities):
                out[nm] = (prev_p.velocities[i]
                           + ratio * (cur_p.velocities[i] - prev_p.velocities[i]))
        return (out['odom_x'], out['odom_y'], out['odom_t'])


class arm_trajectory_action_server(semuInternalState):
    controlled_joints = [
        'arm_lift_joint',
        'arm_flex_joint',
        'arm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
    ]
    # 位置ドライブなので重力方向の姿勢では必ず「たわみ」が残る。許容値を
    # たわみより小さく取ると毎回「目標未達」で中断してしまうため、実測の
    # たわみ (剛性を上げた後で最大 0.02 rad 程度) に対して余裕を持たせる。
    goal_position_tolerance = {
        'arm_lift_joint': 0.01,
        'arm_flex_joint': 0.06,
        'arm_roll_joint': 0.06,
        'wrist_flex_joint': 0.06,
        'wrist_roll_joint': 0.06,
    }
    goal_velocity_tolerance = {
        'arm_lift_joint': 0.015,
        'arm_flex_joint': 0.08,
        'arm_roll_joint': 0.08,
        'wrist_flex_joint': 0.08,
        'wrist_roll_joint': 0.08,
    }
    goal_settle_duration = 0.25
    goal_timeout = 8.0

    def _set_joint_position(self, name: str, target_position: float) -> None:
        if name == 'arm_lift_joint':
            super()._set_joint_position('torso_lift_joint', target_position / 2.0)
        if name in ['arm_flex_joint', 'arm_lift_joint', 'wrist_flex_joint', 'arm_roll_joint']:
            target_position = -target_position
        super()._set_joint_position(name, target_position)

    def _get_joint_position(self, name: str) -> float:
        v = super()._get_joint_position(name)
        if name in ['arm_flex_joint', 'arm_lift_joint', 'wrist_flex_joint', 'arm_roll_joint']:
            return -v
        return v

class head_trajectory_action_server(semuInternalState):
    controlled_joints = ['head_pan_joint', 'head_tilt_joint']


class gripper_trajectory_action_server(semuInternalState):
    controlled_joints = ['hand_motor_joint']

    def _set_joint_position(self, name: str, target_position: float) -> None:
        if name == 'hand_motor_joint':
            super()._set_joint_position('hand_l_proximal_joint', target_position)
            super()._set_joint_position('hand_l_distal_joint', -target_position)
            super()._set_joint_position('hand_r_proximal_joint', target_position)
            super()._set_joint_position('hand_r_distal_joint', -target_position)
        super()._set_joint_position(name, target_position)


class gripper_command_action_server(semuGripperInternalState):
    def __init__(self, hsr, node=None, _dci=None):
        if is_ros2 is False:
            super().__init__()
        else:
            super().__init__(node, _dci)
        self.gripper_joints_paths = [
            hsr.stage_path + '/hsrb/hand_palm_link/hand_l_proximal_joint',
            hsr.stage_path + '/hsrb/hand_l_mimic_distal_link/hand_l_distal_joint',
            hsr.stage_path + '/hsrb/hand_palm_link/hand_r_proximal_joint',
            hsr.stage_path + '/hsrb/hand_r_mimic_distal_link/hand_r_distal_joint',
        ]

    def _set_joint_position(self, name: str, target_position: float) -> None:
        if name == 'hand_l_distal_joint':
            target_position = -target_position
        if name == 'hand_r_distal_joint':
            target_position = -target_position
        super()._set_joint_position(name, target_position)


class gripper_apply_force_action_server(gripper_command_action_server):
    def __init__(self, hsr, node=None, _dci=None):
        if is_ros2 is False:
            super().__init__(hsr)
            self._action_result_message = GripperApplyEffortResult()
            self._action_feedback_message = GripperApplyEffortFeedback()
        else:
            super().__init__(hsr, node, _dci)
            self._action_result_message = GripperApplyEffort.Result()
            self._action_feedback_message = GripperApplyEffort.Feedback()
            # 実機の gripper_controller/grasp, apply_force と同じアクション型で
            # ActionServer を立てる (既定の GripperCommand のままだと型不一致で
            # hsrb_interface クライアントのゴールに応答できず把持が失敗する)。
            self._action_type = GripperApplyEffort
        self._inverse_direction = False
        # 「指が止まった」(stalled) 判定用の連続静止ステップ数カウンタ。
        self._action_stall_steps = 0

    def _get_joint_effort(self, name: str) -> float:
        effort = self.dci.get_dof_state(
            self._joints[name]['dof'], _dynamic_control.STATE_EFFORT
        ).effort
        return effort

    def _get_time(self) -> float:
        if is_ros2:
            return self._node.get_clock().now().nanoseconds / 1e9
        return rospy.get_time()

    def step(self, dt: float) -> None:
        if not self._joints:
            self._init_articulation()
            return
        if self._action_goal is not None and self._action_goal_handle is not None:
            # 結果メッセージはローカルで組み立て、succeed()/abort() を呼んだ後に
            # self._action_result_message へ公開する。途中で公開すると、ros2_bridge の
            # _on_execute スレッド (50ms 周期でループ監視) がゴール状態の設定前に
            # return してしまい、rclpy が "Goal state not set, assuming aborted" と
            # して ABORTED を返すレース条件になる (gripper_close が常に失敗する)。
            # 同じ理由で self._action_goal = None も公開より後にする。
            if is_ros2 is False:
                result_message = GripperApplyEffortResult()
            else:
                result_message = GripperApplyEffort.Result()
            target_effort = self._action_goal.effort
            if self._inverse_direction:
                target_effort = -target_effort

            # 重要: self._joints は _init_articulation が articulation の全DOF
            # (arm/head/wheel 等すべて) を登録する。ここで全DOFに位置0を出すと、
            # gripper_close のたびに腕が原点(arm_lift=0,arm_flex=0)へ畳まれて、把持点から
            # 離れ常に空振りする (実測で確定: close前 arm=(0.5,-1.0)→close後 (0,0))。
            # → グリッパ(hand_*)の指関節だけに限定する。
            gripper_names = [n for n in self._joints if n.startswith('hand_')]
            self.dci.wake_up_articulation(self._articulation)
            for name in gripper_names:
                if target_effort >= 0.0:
                    self._set_joint_position(name, 0.0)
                else:
                    self._set_joint_position(name, math.pi)

            effort = 0
            effort_reached = True
            for name in gripper_names:
                effort = self._get_joint_effort(name)
                if abs(effort) - abs(target_effort) < 0.0:
                    effort_reached = False
                    break
            if effort_reached:
                result_message.effort = effort
                result_message.stalled = False
                if self._action_goal_handle is not None:
                    if is_ros2 is False:
                        self._action_goal_handle.set_succeeded(result_message)
                    else:
                        self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                self._action_result_message = result_message
                self._action_goal = None
                return

            current_position_sum = 0
            for name in gripper_names:
                position = self._get_joint_position(name)
                current_position_sum += position
            # stalled (指が止まった) 判定。元の閾値 1e-6 は Isaac の物理では
            # 永遠に成立しない (閉じ切った後も指は 5e-4〜1e-3/step 程度のクリープ/
            # 微振動を続けると実測) ため、10 秒タイムアウト→ABORTED になっていた。
            # 実測に基づき閾値 2e-3 とし、連続 10 ステップで stalled 確定とする
            # (閉じ動作中は ~2e-2/step なので明確に区別できる)。
            if self._action_previous_position_sum == float('inf'):
                self._action_stall_steps = 0  # 新しいゴールの開始
            pos_diff = abs(current_position_sum -
                           self._action_previous_position_sum)
            if pos_diff < 2e-3:
                self._action_stall_steps += 1
            else:
                self._action_stall_steps = 0
            if self._action_stall_steps >= 10:
                print('[hsr][gripper] stalled -> succeed (effort=%.3f)' % effort)
                result_message.effort = effort
                result_message.stalled = True
                if self._action_goal_handle is not None:
                    if is_ros2 is False:
                        self._action_goal_handle.set_aborted(result_message)
                    else:
                        # stalled = 指が止まった状態。物を掴んで止まるのは正常な把持
                        # 完了なので、実機の gripper controller と同様 succeed を返す
                        # (結果の stalled=True で状態は伝わる)。abort のままだと
                        # hsrb_interface の apply_force が
                        # "Failed to apply force state 6" 例外を投げ、空把持でも
                        # 物を掴んだときでも gripper_close が常に失敗扱いになる。
                        self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                self._action_result_message = result_message
                self._action_goal = None
                return
            self._action_previous_position_sum = current_position_sum

            time_passed = self._get_time() - self._action_start_time
            if time_passed >= self._action_timeout:
                print(
                    '[hsr][gripper] timeout -> abort (stall_steps=%d pos_sum=%.6f time=%.2f)'
                    % (self._action_stall_steps, current_position_sum, time_passed)
                )
                if self._action_goal_handle is not None:
                    if is_ros2 is False:
                        self._action_goal_handle.set_aborted()
                    else:
                        self._action_goal_handle.abort()
                    self._action_goal_handle = None
                self._action_result_message = result_message
                self._action_goal = None


# 案A(アタッチ把持)を使うか。既定は無効で、指と物体の物理接触(摩擦)だけで
# 掴む「正攻法」で動く。シミュレーションとして素直な挙動になり、
# 把持の失敗も物理的に起きるべくして起きる。
#
# 有効にすると物体が指に吸い付き、確実な保持と綺麗なリリースになる。
# 物理調整に手を取られたくないときや、把持より後段を試したいときに使う。
#   make up GRASP=1
_ATTACH_GRASP_ENABLED = os.environ.get('GRASP_ATTACH', '0') != '0'
if not _ATTACH_GRASP_ENABLED:
    print('[graspA] アタッチ把持は無効 (既定)。物理接触のみで把持します。'
          ' 有効にするには make up GRASP=1', flush=True)

# 真値odom: 台車の odom を Isaac 物理の真値(base_footprint 姿勢)で上書きし、計画・制御・TF を
# すべて真値で一致させる。車輪スリップや疎な環境でのレーザ破綻を避けられる(Simだから使える手)。
# 起動時に odometry_switcher を wheel_odom(=この真値) に向ける処理は hsr.launch.py 側。
# False にすると従来の車輪積分 odom に戻る。
_GT_ODOM = True


def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        print(f'[hsr] invalid {name}; using {default}', flush=True)
        return default


def _env_int(name, default):
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        print(f'[hsr] invalid {name}; using {default}', flush=True)
        return default


def _env_bool(name, default):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


# Isaac の車輪 velocity drive は速度目標の急変で揺れやすいので、sim 起動時に
# 環境変数で調整できるようにする。
_BASE_DIRECT_DRIVE = _env_bool('BASE_DIRECT_DRIVE', True)
_BASE_TRAJ_P_GAIN = _env_float('BASE_TRAJ_P_GAIN', 2.0)
_BASE_TRAJ_D_GAIN = max(
    0.0, _env_float('BASE_TRAJ_D_GAIN', 0.5))
_BASE_GOAL_TOLERANCE = max(
    0.0, _env_float('BASE_GOAL_TOLERANCE', 0.010))
_BASE_CONTROL_DEADBAND = max(
    0.0, _env_float('BASE_CONTROL_DEADBAND', 0.003))
# 位置は平面ジョイントのサーボで固定されるので、到達判定は位置誤差で見る。
# 速度の許容値を 0.01 と厳しくすると、腕を動かした反動の微小な揺れだけで
# 「目標未達」となり whole_body 動作が失敗する (実測: 位置誤差 0.8mm でも失敗)。
_BASE_GOAL_VELOCITY_TOLERANCE = max(
    0.0, _env_float('BASE_GOAL_VELOCITY_TOLERANCE', 0.10))
_BASE_GOAL_SETTLE_DURATION = max(
    0.0, _env_float('BASE_GOAL_SETTLE_DURATION', 0.20))
_BASE_GOAL_TIMEOUT = max(
    0.0, _env_float('BASE_GOAL_TIMEOUT', 10.0))
_BASE_CMD_TAU = max(0.0, _env_float('BASE_CMD_TAU', 0.10))
# cmd_vel が途切れてから指令を無効にするまでの猶予。非ゼロ指令のまま送信が
# 途絶えると、この時間ぶん最後の指令で走り続ける。ゼロ指令を受けた場合は
# 速度値の判定で即座に停止するので、この値は関係しない。
_BASE_CMD_VEL_TIMEOUT = max(
    0.0, _env_float('BASE_CMD_VEL_TIMEOUT', 0.3))
_BASE_TRAJ_I_GAIN = max(
    0.0, _env_float('BASE_TRAJ_I_GAIN', 4.0))
_BASE_TRAJ_I_LINEAR_LIMIT = max(
    0.0, _env_float('BASE_TRAJ_I_LINEAR_LIMIT', 0.08))
_BASE_TRAJ_I_ANGULAR_LIMIT = max(
    0.0, _env_float('BASE_TRAJ_I_ANGULAR_LIMIT', 0.15))
_BASE_WHEEL_ACCEL_LIMIT = max(
    0.0, _env_float('BASE_WHEEL_ACCEL_LIMIT', 41.7))
_BASE_STEER_ACCEL_LIMIT = max(
    0.0, _env_float('BASE_STEER_ACCEL_LIMIT', 5.0))
_BASE_LINEAR_ACCEL_LIMIT = max(
    0.0, _env_float('BASE_LINEAR_ACCEL_LIMIT', 0.35))
_BASE_ANGULAR_ACCEL_LIMIT = max(
    0.0, _env_float('BASE_ANGULAR_ACCEL_LIMIT', 0.8))
_BASE_WHEEL_DRIVE_DAMPING = max(
    0.0, _env_float('BASE_WHEEL_DRIVE_DAMPING', 10.0))
# キャスター(受動輪)の転がり軸の粘性摩擦。0 だと軸受けが完全に無摩擦になり、
# 台車が動いたときに床の摩擦で回されたキャスターが、台車が止まった後も
# 慣性で永久に回り続ける (実測: 20秒間 10.06 rad/s から 1e-4 も減らない)。
# 実物の軸受けと同じように減速させるための小さな値。
# BASE_DIRECT_DRIVE=1 (既定) では台車は直接駆動なので走行に影響しないが、
# BASE_DIRECT_DRIVE=0 (物理車輪駆動) では走行抵抗になるので控えめにする。
_BASE_PASSIVE_WHEEL_DAMPING = max(
    0.0, _env_float('BASE_PASSIVE_WHEEL_DAMPING', 0.001))
_BASE_WHEEL_DRIVE_MAX_FORCE = max(
    0.0, _env_float('BASE_WHEEL_DRIVE_MAX_FORCE', 10.0))
_BASE_STEER_DRIVE_DAMPING = max(
    0.0, _env_float('BASE_STEER_DRIVE_DAMPING', 5.0))
_BASE_STEER_DRIVE_MAX_FORCE = max(
    0.0, _env_float('BASE_STEER_DRIVE_MAX_FORCE', 5.0))
_BASE_DIRECT_JOINT_DAMPING = max(
    0.0, _env_float('BASE_DIRECT_JOINT_DAMPING', 1.0))
_BASE_DIRECT_JOINT_MAX_FORCE = max(
    0.0, _env_float('BASE_DIRECT_JOINT_MAX_FORCE', 1.0))
_BASE_DIRECT_ROOT_DAMPING = max(
    0.0, _env_float('BASE_DIRECT_ROOT_DAMPING', 2.0))
# 駐車ブレーキ: 台車が「止まっているべき」間だけ姿勢を固定して腕の反力に
# 流されないようにする (実機のタイヤ摩擦/ブレーキの代役)。直接駆動は摩擦 0 で
# 走るため、速度ゼロ指令だけでは 1 物理ステップ内の反力を止められない。
_BASE_BRAKE_ENABLED = _env_bool('BASE_BRAKE', True)
# ブレーキを掛けてよい残差の上限。これより離れているときは従来の速度制御で寄せる。
_BASE_BRAKE_ENGAGE_LINEAR = max(
    0.0, _env_float('BASE_BRAKE_ENGAGE_LINEAR', 0.05))
_BASE_BRAKE_ENGAGE_ANGULAR = max(
    0.0, _env_float('BASE_BRAKE_ENGAGE_ANGULAR', 0.10))
# ブレーキ中に残差を詰める速さ (瞬間移動を避ける)。
_BASE_BRAKE_CREEP_SPEED = max(
    0.0, _env_float('BASE_BRAKE_CREEP_SPEED', 0.05))
_BASE_BRAKE_CREEP_ANGULAR = max(
    0.0, _env_float('BASE_BRAKE_CREEP_ANGULAR', 0.15))
# 平面ジョイントのドライブ(バネ・ダンパ)のゲイン。台車の保持と走行の両方を
# これで行う。剛性を上げすぎると 60Hz の物理で不安定になるので、ロボット総質量
# 123kg に対し固有振動数 ~40rad/s に収まる値を既定にする。
# 減衰は走行時に抵抗として働くので、C/K 分だけ目標位置を先出しして打ち消す。
_BASE_BRAKE_K = max(0.0, _env_float('BASE_BRAKE_K', 200000.0))
_BASE_BRAKE_C = max(0.0, _env_float('BASE_BRAKE_C', 20000.0))
_BASE_BRAKE_MAX_FORCE = max(0.0, _env_float('BASE_BRAKE_MAX_FORCE', 20000.0))
_BASE_BRAKE_ANGULAR_K = max(0.0, _env_float('BASE_BRAKE_ANGULAR_K', 50000.0))
_BASE_BRAKE_ANGULAR_C = max(0.0, _env_float('BASE_BRAKE_ANGULAR_C', 5000.0))
_BASE_BRAKE_MAX_TORQUE = max(
    0.0, _env_float('BASE_BRAKE_MAX_TORQUE', 5000.0))
# reset_world 直後に台車を強制固定する時間 (秒)。
_BASE_RESET_HOLD_TIME = max(0.0, _env_float('BASE_RESET_HOLD_TIME', 2.0))
# 位置サーボの目標が実位置から離れてよい上限 (アンチワインドアップ)。
_BASE_SETPOINT_MAX_LAG = max(
    0.0, _env_float('BASE_SETPOINT_MAX_LAG', 0.02))
_BASE_SETPOINT_MAX_LAG_ANGULAR = max(
    0.0, _env_float('BASE_SETPOINT_MAX_LAG_ANGULAR', 0.05))
# 平面ジョイントのドライブで台車を保持・駆動する。0 にすると素の速度制御に戻る。
_BASE_JOINT_BRAKE = _env_bool('BASE_JOINT_BRAKE', True)
# 直接駆動時は、平滑化した速度指令を姿勢へ積分し、衝突形状のない kinematic
# anchor を動かす。base_footprint は anchor へ固定するため、内部関節の物理を
# 保ったまま速度書き込みと位置ドライブの競合を避けられる。物理車輪駆動を選んだ
# ときは anchor を作らない。
_BASE_KINEMATIC_DRIVE = (
    _BASE_DIRECT_DRIVE and _env_bool('BASE_KINEMATIC_DRIVE', True))
# 走行中に articulation root の剛体速度を直接書き込むか。
# kinematic 駆動時は「anchor を跳ばした分と同じ速度」を書いて位置と速度を整合
# させるために使う (これを止めると FixedJoint の逆向きインパルスで arm_flex が
# 振られる)。非 kinematic 時は 0 にすると平面ジョイントのドライブ単独になる。
_BASE_VELOCITY_WRITE = _env_bool('BASE_VELOCITY_WRITE', True)
# 純回転のときだけ速度書き込みを省く (回転は角度ドライブ単独で出す)。
_BASE_ROT_NO_VELOCITY_WRITE = _env_bool('BASE_ROT_NO_VELOCITY_WRITE', False)
# kinematic anchor は衝突形状を持たず、base_footprint を breakForce=1e20 の
# FixedJoint で溶接するので、台車は実効的に無限質量になり壁の接触拘束が勝てない
# (壁をすり抜け、PhysX が発散して odom が NaN 化する)。そこで anchor を進める
# 前に PhysX のシーンクエリで球スイープを掛け、貫通する分だけ目標をクランプする。
_BASE_KINEMATIC_COLLISION = _env_bool('BASE_KINEMATIC_COLLISION', True)
# スイープに使う台車の半径 [m]。HSR の台車は半径 0.23m 前後で、前後バンパーを
# 含めた外周を覆うため少し大きめを既定にする。
_BASE_COLLISION_RADIUS = max(
    0.0, _env_float('BASE_COLLISION_RADIUS', 0.24))
# スイープ球の中心高さ [m]。球の下端 (中心 - 半径) が床より上に来る値にする。
# 既定 0.25 なら半径 0.24 の球の下端が z=0.01 になり、床を拾わない。
_BASE_COLLISION_HEIGHT = max(
    0.0, _env_float('BASE_COLLISION_HEIGHT', 0.25))
# 障害物の手前に残す余裕 [m]。0 にすると接触状態で止まる。
_BASE_COLLISION_MARGIN = max(
    0.0, _env_float('BASE_COLLISION_MARGIN', 0.01))
# kinematic anchor をどう動かすか。
#   'usd' : anchor prim の USD xform を毎ステップ書き、PhysX に kinematic target
#           として解釈させる。PhysX が「1ステップかけて目標へ動く」扱いにするので
#           anchor 自身が速度を持ち、FixedJoint で繋がった articulation も滑らかに
#           運ばれる。
#   'dc'  : dc.set_rigid_body_pose で瞬間移動させる (= PxRigidActor::setGlobalPose)。
#           anchor の速度は 0 のままなので、FixedJoint が位置誤差を毎ステップ
#           インパルスで埋めることになり、その衝撃が腕まで伝わって arm_flex が
#           落ちる (実測 1.493 rad)。
# usd が効かない環境 (USD→PhysX の同期が無効など) では台車が動かなくなるので、
# 追従できていないことを検出したら自動的に dc へ戻す。
_BASE_KINEMATIC_ANCHOR_MODE = (
    os.environ.get('BASE_KINEMATIC_ANCHOR_MODE', 'usd').strip().lower()
    or 'usd')
# arm_flex のドライブ力上限 [N*m]。重力たわみを支えるだけなら 100 で足りるが、
# 台車を運ぶ拘束から入る外乱に負けると、maxVelocity=1.2rad/s で飽和したまま
# 落ち続ける。余裕を持たせる。台車は kinematic に位置決めされているので、
# 反力が増えても車体は揺すられない。
_ARM_FLEX_MAX_FORCE = max(
    1.0, _env_float('ARM_FLEX_MAX_FORCE', 300.0))
# 物体が落ち着いた頃に実位置をログへ出す (把持目標を決めるのに使う)。
_OBJECT_POSE_REPORT = _env_bool('OBJECT_POSE_REPORT', True)
_OBJECT_POSE_REPORT_AT = max(
    0.0, _env_float('OBJECT_POSE_REPORT_AT', 5.0))
# 台車の状態を定期的にログへ出す (解析用)。
_BASE_DIAG = _env_bool('BASE_DIAG', False)
# 起動時に物理パラメータ一式をログへ出す (解析用)。
_BASE_PHYSICS_REPORT = _env_bool('BASE_PHYSICS_REPORT', True)
# レンダリングされたフレームのうち何枚に 1 枚を ROS へ publish するか (0 = 毎フレーム)。
# 既定 0: RENDER_EVERY_N_STEPS=2 (=30 Hz レンダリング) と組み合わせて、実機の HSR と同じ
# 30 Hz で /head_rgbd_sensor/* を出す。物理と全身制御は常に 60 Hz のまま。
# 以前は 2 (3 枚に 1 枚 = 5 Hz) だったが、これがそのまま
# /head_rgbd_sensor/reconsted/points が 4 Hz しか出ない原因だった。
# RTF が落ちるようなら 1 (=15 Hz) に戻す。
_CAMERA_FRAME_SKIP = max(0, _env_int('CAMERA_FRAME_SKIP', 0))
# head_l / head_r ステレオ (1280x960 x2) はレンダープロダクトのピクセル予算の 73% を
# 占めるのに、実測で subscriber は 0 だった。RGBD を 30 Hz で出す余力を作るため既定では
# 作らない。ステレオ画像が要るタスクのときだけ ENABLE_STEREO_CAMERAS=1 で復活させる。
_ENABLE_STEREO_CAMERAS = _env_bool('ENABLE_STEREO_CAMERAS', False)


# --- 案A(アタッチ把持)用クォータニオン補助 (x,y,z,w) ---
def _q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_rot(q, v):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class hsr_config:
    def __init__(self) -> None:
        self.use_ros = True


# 台車の駆動系寸法 [m] の既定値(= HSR-B base_v2 の URDF 実寸)。VehicleDynamics
# (速度↔車輪/操舵)と車輪オドメトリが使う。robot ごとに実寸が違うので、実際には
# hsr クラスのクラス属性 WHEEL_* を使い、hsrc_ex は hsr_hsrc_ex 側で上書きする
# (共有コードに片方の robot の値を直書きしない)。
wheel_separation = 0.266   # HSR-B = 2 x drive_wheel_offset_y(0.133)
wheel_radius = 0.04        # HSR-B drive_wheel_radius
wheel_offset = 0.11        # HSR-B drive_wheel_offset_x


class BaseOdometry:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.ang = 0.0


class JointSpace:
    def __init__(self) -> None:
        self.vel_wheel_l = 0.0
        self.vel_wheel_r = 0.0
        self.vel_steer = 0.0


class CartSpace:
    def __init__(self) -> None:
        self.dot_x = 0.0
        self.dot_y = 0.0
        self.dot_r = 0.0


class VehicleState:
    def __init__(self) -> None:
        self.steer_angle = 0.0


class VehicleDynamics:
    """
    Dynamics of offset diff drive vehicle
    Equations are from the paper written by Masayoshi Wada etal.
    https://www.jstage.jst.go.jp/article/jrsj1983/18/8/18_8_1166/_pdf
    """

    def __init__(self, wheel_radius: float, wheel_separation: float, wheel_offset: float) -> None:
        self._wheel_radius = wheel_radius
        self._wheel_separation = wheel_separation
        self._wheel_offset = wheel_offset

    def forward(self, joint_space: JointSpace, state: VehicleState) -> CartSpace:
        cos_s = math.cos(state.steer_angle)
        sin_s = math.sin(state.steer_angle)
        output = CartSpace()
        output.dot_x = (
            self._wheel_radius / 2.0 * cos_s
            - self._wheel_radius * self._wheel_offset / self._wheel_separation * sin_s
        ) * joint_space.vel_wheel_r + (
            self._wheel_radius / 2.0 * cos_s
            + self._wheel_radius * self._wheel_offset / self._wheel_separation * sin_s
        ) * joint_space.vel_wheel_l
        output.dot_y = (
            self._wheel_radius / 2.0 * sin_s
            + self._wheel_radius * self._wheel_offset / self._wheel_separation * cos_s
        ) * joint_space.vel_wheel_r + (
            self._wheel_radius / 2.0 * sin_s
            - self._wheel_radius * self._wheel_offset / self._wheel_separation * cos_s
        ) * joint_space.vel_wheel_l
        output.dot_r = (
            self._wheel_radius / self._wheel_separation * joint_space.vel_wheel_r
            - self._wheel_radius / self._wheel_separation * joint_space.vel_wheel_l
            - joint_space.vel_steer
        )
        return output

    def inverse(self, cart_space: CartSpace, state: VehicleState) -> JointSpace:
        cos_s = math.cos(state.steer_angle)
        sin_s = math.sin(state.steer_angle)
        output = JointSpace()
        output.vel_wheel_r = (
            cos_s / self._wheel_radius
            - self._wheel_separation * sin_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_x + (
            sin_s / self._wheel_radius
            + self._wheel_separation * cos_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_y
        output.vel_wheel_l = (
            cos_s / self._wheel_radius
            + self._wheel_separation * sin_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_x + (
            sin_s / self._wheel_radius
            - self._wheel_separation * cos_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_y
        output.vel_steer = (
            -sin_s / self._wheel_offset * cart_space.dot_x
            + cos_s / self._wheel_offset * cart_space.dot_y
            - cart_space.dot_r
        )
        return output


class WheelOdometry:
    def __init__(self) -> None:
        self.pose = BaseOdometry()

    def integrate(self, cart_velocity: CartSpace, dt: float):
        diff_r = cart_velocity.dot_r * dt
        cosr = math.cos(self.pose.ang + 0.5 * diff_r)
        sinr = math.sin(self.pose.ang + 0.5 * diff_r)
        world_dot_x = cart_velocity.dot_x * cosr - cart_velocity.dot_y * sinr
        world_dot_y = cart_velocity.dot_x * sinr + cart_velocity.dot_y * cosr
        if dt > 0.0:
            self.pose.x += world_dot_x * dt
            self.pose.y += world_dot_y * dt
            self.pose.ang += diff_r
        return world_dot_x, world_dot_y

    def set_pose(self, x: float, y: float, ang: float) -> None:
        self.pose.x = x
        self.pose.y = y
        self.pose.ang = ang


class hsr:
    # 台車の駆動系寸法 [m]。既定は HSR-B(base_v2)。hsrc_ex は hsr_hsrc_ex.hsr が
    # これらを上書きする(URDF base_v0 の実寸)。VehicleDynamics と車輪オドメトリが使う。
    WHEEL_RADIUS = wheel_radius
    WHEEL_SEPARATION = wheel_separation
    WHEEL_OFFSET = wheel_offset

    def __init__(self, prefix='/hsrb', stage_path='/World', config=None) -> None:
        global is_ros2

        if config is None:
            config = hsr_config()

        # if is_ros2:
        #    rclpy.init()
        #    self.ros2node = rclpy.node.Node("isaac_sim_hsr")
        #    self.create_subscriber = lambda t, d, c: self.ros2node.create_subscription(d, t, c, qos_profile=rclpy.qos.qos_profile_sensor_data)
        #    self.create_publisher = lambda t, d: self.ros2node.create_publisher(d, t, qos_profile=rclpy.qos.qos_profile_sensor_data)
        #    self.create_publisher_reliable = lambda t, d: self.ros2node.create_publisher(d, t, qos_profile=rclpy.qos.qos_profile_system_default)
        #    self.get_ros_time = lambda t: rclpy.time.Time(seconds=t).to_msg()
        #    self.tf_broadcaster = TransformBroadcaster(self.ros2node)
        #    self._create_controller_parameter_services()
        #    executor = rclpy.executors.MultiThreadedExecutor()
        #    executor.add_node(self.ros2node)
        #    threading.Thread(target=executor.spin).start()
        # else:
        #    # add by r.kobayashi
        #    try:
        #        rospy.init_node("isaac_sim_hsr", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
        #    except rospy.exception.ROSException:
        #        pass

        #    self.create_subscriber = lambda t, d, c: rospy.Subscriber(t, d, c)
        #    self.create_publisher = lambda t, d: rospy.Publisher(t, d, queue_size=5)
        #    self.create_publisher_reliable = lambda t, d: rospy.Publisher(t, d, queue_size=5)
        #    self.get_ros_time = lambda t: rospy.Time(t)
        if is_ros2:
            if not rclpy.ok():
                rclpy.init()
            self.ros2node = rclpy.node.Node('isaac_sim_hsr')
            # Trajectory time_from_start is expressed in simulation time.  The
            # simulator can run below real time, so a wall-clock controller
            # advances targets too quickly relative to PhysX and excites the
            # mobile base.
            self.ros2node.set_parameters([
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ])
            self.create_subscriber = lambda t, d, c: self.ros2node.create_subscription(
                d, t, c, qos_profile=rclpy.qos.qos_profile_sensor_data
            )
            self.create_publisher = lambda t, d: self.ros2node.create_publisher(
                d, t, qos_profile=rclpy.qos.qos_profile_sensor_data
            )
            self.create_publisher_reliable = lambda t, d: self.ros2node.create_publisher(
                d, t, qos_profile=rclpy.qos.qos_profile_system_default
            )
            self.clock_pub = self.ros2node.create_publisher(
                Clock, '/clock', 1)
            self.get_ros_time = lambda t: rclpy.time.Time(seconds=t).to_msg()
            self.tf_broadcaster = TransformBroadcaster(self.ros2node)
            self._create_controller_parameter_services()

            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(self.ros2node)
            self._executor = executor
            self._executor_thread = threading.Thread(
                target=executor.spin, daemon=True)
            self._executor_thread.start()
        else:
            try:
                rospy.init_node(
                    'isaac_sim_hsr', anonymous=True, disable_signals=True, log_level=rospy.ERROR
                )
            except rospy.exceptions.ROSException:
                pass

            self.create_subscriber = lambda t, d, c: rospy.Subscriber(t, d, c)
            self.create_publisher = lambda t, d: rospy.Publisher(
                t, d, queue_size=5)
            self.create_publisher_reliable = lambda t, d: rospy.Publisher(
                t, d, queue_size=5)
            self.get_ros_time = lambda t: rospy.Time.from_sec(t)

        self.prefix = prefix
        self.stage_path = stage_path
        self.simulation_context = None
        self.art = None
        # HSR モデル(usd)の場所を、配置に依らず見つける。
        #   - 焼き込みフラット配置: /app/hsr.py    → /app/usd/hsrb/hsrb4s.usd
        #   - リポジトリ配置:       /app/scripts/hsr.py → /app/usd/hsrb/hsrb4s.usd
        #     (usd は scripts の隣ではなくリポジトリ直下にあるため '..' を見る)
        _here = os.path.dirname(os.path.abspath(__file__))
        _hsr_usd_candidates = [
            os.path.join(_here, 'usd', 'hsrb', 'hsrb4s.usd'),
            os.path.join(_here, '..', 'usd', 'hsrb', 'hsrb4s.usd'),
            '/app/usd/hsrb/hsrb4s.usd',
        ]
        _hsr_usd = next(
            (p for p in _hsr_usd_candidates if os.path.exists(p)),
            _hsr_usd_candidates[0],
        )
        self.hsr = stage.add_reference_to_stage(
            _hsr_usd,
            self.stage_path + self.prefix,
        )
        self.set_base_joint_and_material()

        self.create_cameras()
        self.create_lidar()
        self.create_imu()

        if is_ros2:
            self.laserscan_pose_sub = self.create_subscriber(
                '/laser_odom', Odometry, self.on_laserscan_odom
            )
            self.base_odom_pub = self.create_publisher_reliable(
                '/omni_base_controller/wheel_odom', Odometry
            )
        else:
            self.laserscan_pose_sub = self.create_subscriber(
                self.prefix + '/laser_scan_matcher/pose', PoseStamped, self.on_laserscan_pose
            )
            self.laser_odom_pub = self.create_publisher(
                self.prefix + '/laser_odom', Odometry)
            self.base_odom_pub = self.create_publisher('/odom', Odometry)

        self.robots = ArticulationView(
            prim_paths_expr=self.stage_path + self.prefix, name='hsr_view'
        )
        # wrench は RELIABLE で出す。実機の /hsrb/wrist_wrench/* は RELIABLE で、
        # skill の is_hand_collision は rclpy.wait_for_message を既定QoS(RELIABLE)で
        # 購読する。BEST_EFFORT で出すと QoS 不一致でメッセージが届かず
        # wait_for_message が永久ブロックする (実測)。
        self.ft_sensor_pub = self.create_publisher_reliable(
            self.prefix + '/wrist_wrench/raw', WrenchStamped
        )
        # 重力補正済み wrench。実機は compensated を出すので skill は
        # こちらを優先購読する。EMA で重力(=ゆっくり変化)を差し引き、
        # 接触の過渡だけ残す。
        self.ft_sensor_comp_pub = self.create_publisher_reliable(
            self.prefix + '/wrist_wrench/compensated', WrenchStamped
        )
        self._wrench_bias = [0.0] * 6
        self._wrench_bias_inited = False

        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        self.cmd_vel_msg = None
        self.last_cmd_vel_time = 0.0
        self.create_subscriber(
            '/omni_base_controller/cmd_vel' if is_ros2 else self.prefix + '/command_velocity',
            Twist,
            self.on_cmd_vel,
        )
        self.vehicle_dynamics = VehicleDynamics(
            self.WHEEL_RADIUS, self.WHEEL_SEPARATION, self.WHEEL_OFFSET)
        self.odometry_estimator = WheelOdometry()
        # Match hsrb_base_controllers.  Scaling all three axes by the most
        # restrictive limit preserves the twin-caster kinematic ratio.
        self.vel_limit_steer_ = 1.8
        self.vel_limit_wheel_ = 8.5
        self._base_cmd_prev = JointSpace()
        self._base_direct_cmd_prev = CartSpace()
        self._base_direct_applied_cmd = CartSpace()
        self._base_direct_pose = None
        self._base_kinematic_velocity = (0.0, 0.0, 0.0)
        self._base_hold_pose = None
        self._base_brake_engaged = False
        self._base_brake_pin = None
        self._base_setpoint = None
        self._base_pose_error_integral = [0.0, 0.0, 0.0]
        self._base_trajectory_was_active = False

        self.joint_state_pub = self.create_publisher_reliable(
            '/joint_states' if is_ros2 else self.prefix + '/joint_states', JointState
        )

        if is_ros2 is False:

            def init_action_server(srv, name, msg):
                srv.articulation_path = self.stage_path + self.prefix
                srv.usd_context = omni.usd.get_context()
                srv.dci = self.dc
                action_topic_name = self.prefix + '/' + name
                srv.action_server = actionlib.ActionServer(
                    action_topic_name,
                    msg,
                    goal_cb=srv.on_goal,
                    cancel_cb=srv.on_cancel,
                    auto_start=False,
                )
                srv.action_server.start()
                if msg == FollowJointTrajectoryAction:
                    srv.action_client = actionlib.SimpleActionClient(
                        action_topic_name, msg)

                    def command_topic_callback(msg, args):
                        srv = args[0]
                        goal = FollowJointTrajectoryGoal(trajectory=msg)
                        if srv._action_goal is not None:
                            for name in goal.trajectory.joint_names:
                                if name not in self._joints:
                                    print(
                                        "[Warning][semu.robotics.ros_bridge] ROS1 FollowJointTrajectory: joints don't match ({} not in {})".format(
                                            name, list(self._joints.keys())
                                        )
                                    )
                                    return
                            if goal.trajectory.points[0].time_from_start.to_sec():
                                initial_point = JointTrajectoryPoint(
                                    positions=[
                                        srv._get_joint_position(name)
                                        for name in goal.trajectory.joint_names
                                    ],
                                    time_from_start=rospy.Duration(),
                                )
                                goal.trajectory.points.insert(0, initial_point)
                            srv._action_goal = goal
                            srv._action_point_index = 1
                            srv._action_start_time = rospy.get_time()
                            srv._action_feedback_message.joint_names = list(
                                goal.trajectory.joint_names
                            )
                        else:
                            srv.action_client.send_goal(goal)

                    srv.topic_interface = rospy.Subscriber(
                        action_topic_name.replace(
                            '/follow_joint_trajectory', '/command'),
                        JointTrajectory,
                        command_topic_callback,
                        (srv,),
                    )
                srv.initialized = True

            self.arm_trajectory_action_server = arm_trajectory_action_server()
            init_action_server(
                self.arm_trajectory_action_server,
                'arm_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
            )

            self.head_trajectory_action_server = head_trajectory_action_server()
            init_action_server(
                self.head_trajectory_action_server,
                'head_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
            )

            self.odom_trajectory_action_server = odom_trajectory_action_server()
            init_action_server(
                self.odom_trajectory_action_server,
                'omni_base_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
            )

            self.gripper_trajectory_action_server = gripper_trajectory_action_server()
            init_action_server(
                self.gripper_trajectory_action_server,
                'gripper_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction,
            )

            self.gripper_apply_force_action_server = gripper_apply_force_action_server(
                self)
            init_action_server(
                self.gripper_apply_force_action_server,
                'gripper_controller/apply_force',
                GripperApplyEffortAction,
            )

            self.gripper_command_action_server = gripper_apply_force_action_server(
                self)
            self.gripper_command_action_server._inverse_direction = True
            init_action_server(
                self.gripper_command_action_server,
                'gripper_controller/grasp',
                GripperApplyEffortAction,
            )
        else:

            def init_action_server(srv, name, msg):
                articulation_namespace = self.stage_path + self.prefix
                action_topic_name = '/' + name
                srv.start(articulation_namespace, action_topic_name)

            self.arm_trajectory_action_server = arm_trajectory_action_server(
                self.ros2node, self.dc)
            init_action_server(
                self.arm_trajectory_action_server,
                'arm_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectory,
            )

            self.head_trajectory_action_server = head_trajectory_action_server(
                self.ros2node, self.dc
            )
            init_action_server(
                self.head_trajectory_action_server,
                'head_trajectory_controller/follow_joint_trajectory',
                FollowJointTrajectory,
            )

            self.odom_trajectory_action_server = odom_trajectory_action_server(
                self.ros2node, self.dc
            )
            init_action_server(
                self.odom_trajectory_action_server,
                'omni_base_controller/follow_joint_trajectory',
                FollowJointTrajectory,
            )

            self.gripper_trajectory_action_server = gripper_trajectory_action_server(
                self.ros2node, self.dc
            )
            init_action_server(
                self.gripper_trajectory_action_server,
                'gripper_controller/follow_joint_trajectory',
                FollowJointTrajectory,
            )

            self.gripper_apply_force_action_server = gripper_apply_force_action_server(
                self, self.ros2node, self.dc
            )
            init_action_server(
                self.gripper_apply_force_action_server,
                'gripper_controller/apply_force',
                GripperApplyEffort,
            )

            self.gripper_command_action_server = gripper_apply_force_action_server(
                self, self.ros2node, self.dc
            )
            self.gripper_command_action_server._inverse_direction = True
            init_action_server(
                self.gripper_command_action_server, 'gripper_controller/grasp', GripperApplyEffort
            )

    def create_cameras(self) -> None:
        topic_prefix = '' if is_ros2 else self.prefix
        print(
            '[hsr-camera] publish every %d rendered frame(s) '
            '(CAMERA_FRAME_SKIP=%d) stereo=%s'
            % (_CAMERA_FRAME_SKIP + 1, _CAMERA_FRAME_SKIP,
               'on' if _ENABLE_STEREO_CAMERAS else 'off'),
            flush=True,
        )

        # ステレオは既定 off (ENABLE_STEREO_CAMERAS=1 で on)。Camera prim を作らなければ
        # レンダープロダクトも OG グラフも作られないので、GPU のラスタライズも
        # ROS シリアライズもまるごと消える。
        if _ENABLE_STEREO_CAMERAS:
            l_camera_prim = UsdGeom.Camera(
                omni.usd
                .get_context()
                .get_stage()
                .DefinePrim(
                    self.stage_path + self.prefix + '/head_l_stereo_camera_link/Camera', 'Camera'
                )
            )
            xform_api = UsdGeom.XformCommonAPI(l_camera_prim)
            xform_api.SetRotate(
                (180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            l_camera_prim.GetHorizontalApertureAttr().Set(1280 * 0.003)
            l_camera_prim.GetVerticalApertureAttr().Set(960 * 0.003)
            l_camera_prim.GetProjectionAttr().Set('perspective')
            l_camera_prim.GetFocalLengthAttr().Set(968.770306867 * 0.003)
            l_camera_prim.GetFocusDistanceAttr().Set(400)
            # near=0.07m, far=100m。既定near=1mだと1m以内の近接物体が消えるので小さくする。
            # 頭部RGBDの自己オクルージョン対策と値を揃え、全カメラ0.07mに統一。far=100mは室内に十分。
            l_camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.07, 100.0))

            r_camera_prim = UsdGeom.Camera(
                omni.usd
                .get_context()
                .get_stage()
                .DefinePrim(
                    self.stage_path + self.prefix + '/head_r_stereo_camera_link/Camera', 'Camera'
                )
            )
            xform_api = UsdGeom.XformCommonAPI(r_camera_prim)
            xform_api.SetRotate(
                (180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            r_camera_prim.GetHorizontalApertureAttr().Set(1280 * 0.003)
            r_camera_prim.GetVerticalApertureAttr().Set(960 * 0.003)
            r_camera_prim.GetProjectionAttr().Set('perspective')
            r_camera_prim.GetFocalLengthAttr().Set(968.770306867 * 0.003)
            r_camera_prim.GetFocusDistanceAttr().Set(400)
            # near=0.07m, far=100m。既定near=1mだと1m以内の近接物体が消えるので小さくする。
            # 頭部RGBDの自己オクルージョン対策と値を揃え、全カメラ0.07mに統一。far=100mは室内に十分。
            r_camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.07, 100.0))

        rgbd_camera_prim = UsdGeom.Camera(
            omni.usd
            .get_context()
            .get_stage()
            .DefinePrim(self.stage_path + self.prefix + '/head_rgbd_sensor_link/Camera', 'Camera')
        )
        xform_api = UsdGeom.XformCommonAPI(rgbd_camera_prim)
        xform_api.SetRotate(
            (180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        rgbd_camera_prim.GetHorizontalApertureAttr().Set(640 * 0.003)
        rgbd_camera_prim.GetVerticalApertureAttr().Set(480 * 0.003)
        rgbd_camera_prim.GetProjectionAttr().Set('perspective')
        rgbd_camera_prim.GetFocalLengthAttr().Set(554.382712823 * 0.003)
        rgbd_camera_prim.GetFocusDistanceAttr().Set(400)
        # near=0.07m。nearを極小(1cm)にすると、頭部RGBDの広い視野では視界の上側に
        # ロボット自身の頭/体が映り込み「画像の上半分が黒い帯」になる(自己オクルージョン)。
        # 一方で既定の1mだと近接物体が消える。0.07mは近接物体を残しつつ自分の体を切る妥協点(実測)。
        # far=100mは室内に十分(深度精度も問題なし)。
        rgbd_camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.07, 100.0))

        hand_camera_prim = UsdGeom.Camera(
            omni.usd
            .get_context()
            .get_stage()
            .DefinePrim(self.stage_path + self.prefix + '/hand_camera_frame/Camera', 'Camera')
        )
        xform_api = UsdGeom.XformCommonAPI(hand_camera_prim)
        xform_api.SetRotate(
            (180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        hand_camera_prim.GetHorizontalApertureAttr().Set(640 * 0.003)
        hand_camera_prim.GetVerticalApertureAttr().Set(480 * 0.003)
        hand_camera_prim.GetProjectionAttr().Set('perspective')
        hand_camera_prim.GetFocalLengthAttr().Set(205.469637099 * 0.003)
        hand_camera_prim.GetFocusDistanceAttr().Set(400)
        # near=0.01m, far=100m。ハンドカメラだけ他より近くする。
        # 他カメラと同じ 0.07m だと、カメラから 6.2cm にある指の付け根が
        # クリップされて消え、指が途中から生えて見える (切断面の向こうが透ける)。
        #   カメラ -> proximal 0.062m / distal 0.083m / 指先 0.103m
        hand_camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))

        # ステレオ無効時も属性自体は必ず生えている状態にしておく (AttributeError 回避)。
        self.ros_camera_graph_l = None
        self.ros_camera_graph_r = None

        try:
            og.Controller.edit(
                {'graph_path': '/ros_controllers', 'evaluator_name': 'execution'},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnImpulseEvent', 'omni.graph.action.OnImpulseEvent'),
                        ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                        ('PublishClock', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1PublishClock')),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ('OnImpulseEvent.outputs:execOut',
                         'PublishClock.inputs:execIn'),
                        ('ReadSimTime.outputs:simulationTime',
                         'PublishClock.inputs:timeStamp'),
                    ],
                    og.Controller.Keys.SET_VALUES: [],
                },
            )

            # ステレオ 2 本のグラフ (レンダープロダクト 1280x960 x2 込み) は既定では作らない。
            if _ENABLE_STEREO_CAMERAS:
                (self.ros_camera_graph_l, _, _, _) = og.Controller.edit(
                    {
                        'graph_path': '/head_l_camera',
                        'evaluator_name': 'push',
                        'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                    },
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ('OnTick', 'omni.graph.action.OnTick'),
                            ('createRenderProduct',
                             'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                            ('cameraHelperRgb', og_ros_node(
                                'isaacsim.ros1.bridge.ROS1CameraHelper')),
                            ('cameraHelperInfo', og_ros_node(
                                'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ('OnTick.outputs:tick', 'createRenderProduct.inputs:execIn'),
                            ('createRenderProduct.outputs:execOut',
                             'cameraHelperRgb.inputs:execIn'),
                            ('createRenderProduct.outputs:execOut',
                             'cameraHelperInfo.inputs:execIn'),
                            (
                                'createRenderProduct.outputs:renderProductPath',
                                'cameraHelperRgb.inputs:renderProductPath',
                            ),
                            (
                                'createRenderProduct.outputs:renderProductPath',
                                'cameraHelperInfo.inputs:renderProductPath',
                            ),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ('createRenderProduct.inputs:width', 1280),
                            ('createRenderProduct.inputs:height', 960),
                            ('cameraHelperRgb.inputs:frameId',
                             'head_l_stereo_camera_frame'),
                            (
                                'cameraHelperRgb.inputs:topicName',
                                topic_prefix + '/head_l_stereo_camera/image_rect_color',
                            ),
                            ('cameraHelperRgb.inputs:frameSkipCount',
                             _CAMERA_FRAME_SKIP),
                            ('cameraHelperRgb.inputs:type', 'rgb'),
                            ('cameraHelperInfo.inputs:frameId',
                             'head_l_stereo_camera_frame'),
                            (
                                'cameraHelperInfo.inputs:topicName',
                                topic_prefix + '/head_l_stereo_camera/camera_info',
                            ),
                            ('cameraHelperInfo.inputs:frameSkipCount',
                             _CAMERA_FRAME_SKIP),
                            ('cameraHelperInfo.inputs:type', 'camera_info'),
                        ],
                    },
                )

                (self.ros_camera_graph_r, _, _, _) = og.Controller.edit(
                    {
                        'graph_path': '/head_r_camera',
                        'evaluator_name': 'push',
                        'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                    },
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ('OnTick', 'omni.graph.action.OnTick'),
                            ('createRenderProduct',
                             'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                            ('cameraHelperRgb', og_ros_node(
                                'isaacsim.ros1.bridge.ROS1CameraHelper')),
                            ('cameraHelperInfo', og_ros_node(
                                'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ('OnTick.outputs:tick', 'createRenderProduct.inputs:execIn'),
                            ('createRenderProduct.outputs:execOut',
                             'cameraHelperRgb.inputs:execIn'),
                            ('createRenderProduct.outputs:execOut',
                             'cameraHelperInfo.inputs:execIn'),
                            (
                                'createRenderProduct.outputs:renderProductPath',
                                'cameraHelperRgb.inputs:renderProductPath',
                            ),
                            (
                                'createRenderProduct.outputs:renderProductPath',
                                'cameraHelperInfo.inputs:renderProductPath',
                            ),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ('createRenderProduct.inputs:width', 1280),
                            ('createRenderProduct.inputs:height', 960),
                            ('cameraHelperRgb.inputs:frameId',
                             'head_r_stereo_camera_frame'),
                            (
                                'cameraHelperRgb.inputs:topicName',
                                topic_prefix + '/head_r_stereo_camera/image_rect_color',
                            ),
                            ('cameraHelperRgb.inputs:frameSkipCount',
                             _CAMERA_FRAME_SKIP),
                            ('cameraHelperRgb.inputs:type', 'rgb'),
                            ('cameraHelperInfo.inputs:frameId',
                             'head_r_stereo_camera_frame'),
                            (
                                'cameraHelperInfo.inputs:topicName',
                                topic_prefix + '/head_r_stereo_camera/camera_info',
                            ),
                            ('cameraHelperInfo.inputs:frameSkipCount',
                             _CAMERA_FRAME_SKIP),
                            ('cameraHelperInfo.inputs:type', 'camera_info'),
                        ],
                    },
                )

            (self.ros_camera_graph_rgbd, _, _, _) = og.Controller.edit(
                {
                    'graph_path': '/head_rgbd_camera',
                    'evaluator_name': 'push',
                    'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnTick', 'omni.graph.action.OnTick'),
                        ('createRenderProduct',
                         'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                        ('cameraHelperRgb', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ('cameraHelperInfo', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ('cameraHelperDepth', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        (
                            'cameraHelperDepthInfo',
                            og_ros_node(
                                'isaacsim.ros1.bridge.ROS1CameraHelper'),
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ('OnTick.outputs:tick', 'createRenderProduct.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperRgb.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperInfo.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperDepth.inputs:execIn'),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperRgb.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperInfo.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperDepth.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperDepthInfo.inputs:renderProductPath',
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ('createRenderProduct.inputs:width', 640),
                        ('createRenderProduct.inputs:height', 480),
                        ('cameraHelperRgb.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperRgb.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/rgb/image_rect_color',
                        ),
                        ('cameraHelperRgb.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/rgb/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperInfo.inputs:type', 'camera_info'),
                        ('cameraHelperDepth.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperDepth.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/depth_registered/image_rect_raw',
                        ),
                        ('cameraHelperDepth.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperDepth.inputs:type', 'depth'),
                        # Sensor Data QoS (BEST_EFFORT) so the depth image
                        # propagates over CycloneDDS PC unicast to remote
                        # subscribers (e.g. pumas_navigation on a separate PC).
                        # Keys must use camelCase (keepLast/bestEffort) per
                        # Isaac Sim 4.5 OgnROS2QoSProfile schema.
                        (
                            'cameraHelperDepth.inputs:qosProfile',
                            '{"history":"keepLast","depth":5,"reliability":"bestEffort","durability":"volatile","deadline":0.0,"lifespan":0.0,"liveliness":"systemDefault","leaseDuration":0.0}',
                        ),
                        ('cameraHelperDepthInfo.inputs:frameId',
                         'head_rgbd_sensor_rgb_frame'),
                        (
                            'cameraHelperDepthInfo.inputs:topicName',
                            topic_prefix + '/head_rgbd_sensor/depth_registered/camera_info',
                        ),
                        ('cameraHelperDepthInfo.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperDepthInfo.inputs:type', 'camera_info'),
                        # camera_info stays RELIABLE (default) — small payload,
                        # and depth_image_proc::PointCloudXyzrgbNode subscribes
                        # via image_transport/message_filters which does NOT
                        # honor qos_overrides parameters. Keeping RELIABLE
                        # avoids the QoS mismatch that blocks pointcloud output.
                    ],
                },
            )

            (self.ros_camera_graph_hand, _, _, _) = og.Controller.edit(
                {
                    'graph_path': '/hand_camera',
                    'evaluator_name': 'push',
                    'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ('OnTick', 'omni.graph.action.OnTick'),
                        ('createRenderProduct',
                         'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                        ('cameraHelperRgb', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                        ('cameraHelperInfo', og_ros_node(
                            'isaacsim.ros1.bridge.ROS1CameraHelper')),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ('OnTick.outputs:tick', 'createRenderProduct.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperRgb.inputs:execIn'),
                        ('createRenderProduct.outputs:execOut',
                         'cameraHelperInfo.inputs:execIn'),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperRgb.inputs:renderProductPath',
                        ),
                        (
                            'createRenderProduct.outputs:renderProductPath',
                            'cameraHelperInfo.inputs:renderProductPath',
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ('createRenderProduct.inputs:width', 640),
                        ('createRenderProduct.inputs:height', 480),
                        ('cameraHelperRgb.inputs:frameId', 'hand_camera_frame'),
                        (
                            'cameraHelperRgb.inputs:topicName',
                            topic_prefix + '/hand_camera/image_raw',
                        ),
                        ('cameraHelperRgb.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperRgb.inputs:type', 'rgb'),
                        ('cameraHelperInfo.inputs:frameId', 'hand_camera_frame'),
                        (
                            'cameraHelperInfo.inputs:topicName',
                            topic_prefix + '/hand_camera/camera_info',
                        ),
                        ('cameraHelperInfo.inputs:frameSkipCount',
                         _CAMERA_FRAME_SKIP),
                        ('cameraHelperInfo.inputs:type', 'camera_info'),
                    ],
                },
            )
        except Exception as e:
            raise e

        if _ENABLE_STEREO_CAMERAS:
            set_targets(
                prim=stage.get_current_stage().GetPrimAtPath(
                    '/head_l_camera/createRenderProduct'),
                attribute='inputs:cameraPrim',
                target_prim_paths=[self.stage_path + self.prefix +
                                   '/head_l_stereo_camera_link/Camera'],
            )

            set_targets(
                prim=stage.get_current_stage().GetPrimAtPath(
                    '/head_r_camera/createRenderProduct'),
                attribute='inputs:cameraPrim',
                target_prim_paths=[self.stage_path + self.prefix +
                                   '/head_r_stereo_camera_link/Camera'],
            )

        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(
                '/head_rgbd_camera/createRenderProduct'),
            attribute='inputs:cameraPrim',
            target_prim_paths=[self.stage_path +
                               self.prefix + '/head_rgbd_sensor_link/Camera'],
        )

        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(
                '/hand_camera/createRenderProduct'),
            attribute='inputs:cameraPrim',
            target_prim_paths=[self.stage_path +
                               self.prefix + '/hand_camera_frame/Camera'],
        )

        if _ENABLE_STEREO_CAMERAS:
            og.Controller.evaluate_sync(self.ros_camera_graph_l)
            og.Controller.evaluate_sync(self.ros_camera_graph_r)
        og.Controller.evaluate_sync(self.ros_camera_graph_rgbd)
        og.Controller.evaluate_sync(self.ros_camera_graph_hand)

    def create_lidar_rtx(self) -> None:
        lidar_config = 'Example_Rotary'
        _, sensor = omni.kit.commands.execute(
            'IsaacSensorCreateRtxLidar',
            path=self.stage_path + self.prefix + '/base_range_sensor_link/Lidar',
            parent=None,
            config=lidar_config,
        )
        _, render_product_path = create_hydra_texture(
            [1, 1], sensor.GetPath().pathString)
        writer = rep.writers.get('RtxLidar' + 'DebugDrawPointCloud')
        writer.attach([render_product_path])
        writer = rep.writers.get('RtxLidar' + 'ROS1PublishPointCloud')
        writer.attach([render_product_path])

    def create_imu(self) -> None:
        _, sensor = omni.kit.commands.execute(
            'IsaacSensorCreateImuSensor',
            path=self.stage_path + self.prefix + '/base_imu_frame/Imu_Sensor',
            parent=None,
            sensor_period=-1,
        )
        (self.ros_imu, _, _, _) = og.Controller.edit(
            {
                'graph_path': self.stage_path + self.prefix + '/imu_sensor',
                'evaluator_name': 'execution',
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                    ('readSimulationTime',
                     'isaacsim.core.nodes.IsaacReadSimulationTime'),
                    ('readImu', 'isaacsim.sensors.physics.IsaacReadIMU'),
                    ('publishImu', og_ros_node('isaacsim.ros1.bridge.ROS1PublishImu')),
                ],
                og.Controller.Keys.CONNECT: [
                    ('OnTick.outputs:tick', 'readImu.inputs:execIn'),
                    ('readImu.outputs:execOut', 'publishImu.inputs:execIn'),
                    ('readSimulationTime.outputs:simulationTime',
                     'publishImu.inputs:timeStamp'),
                    ('readImu.outputs:linAcc',
                     'publishImu.inputs:linearAcceleration'),
                    ('readImu.outputs:angVel', 'publishImu.inputs:angularVelocity'),
                    ('readImu.outputs:orientation',
                     'publishImu.inputs:orientation'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('publishImu.inputs:frameId', 'base_imu_frame'),
                    ('publishImu.inputs:topicName',
                     self.prefix + '/base_imu/data'),
                ],
            },
        )
        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(
                self.stage_path + self.prefix + '/imu_sensor/readImu'
            ),
            attribute='inputs:imuPrim',
            target_prim_paths=[self.stage_path +
                               self.prefix + '/base_imu_frame/Imu_Sensor'],
        )

    def create_lidar(self) -> None:
        _, sensor = omni.kit.commands.execute(
            'RangeSensorCreateLidar',
            path=self.stage_path + self.prefix + '/base_range_sensor_link/Lidar',
            parent=None,
            min_range=0.3,
            max_range=60.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=240.0,
            horizontal_resolution=0.25,
            rotation_rate=30,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )
        (self.ros_lidar, _, _, _) = og.Controller.edit(
            {
                'graph_path': '/lidar_sensor',
                'evaluator_name': 'execution',
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                    ('readSimulationTime',
                     'isaacsim.core.nodes.IsaacReadSimulationTime'),
                    ('readLidarBeams', 'isaacsim.sensors.physx.IsaacReadLidarBeams'),
                    ('publishLaserScan', og_ros_node(
                        'isaacsim.ros1.bridge.ROS1PublishLaserScan')),
                ],
                og.Controller.Keys.CONNECT: [
                    ('OnTick.outputs:tick', 'readLidarBeams.inputs:execIn'),
                    ('readLidarBeams.outputs:execOut',
                     'publishLaserScan.inputs:execIn'),
                    (
                        'readSimulationTime.outputs:simulationTime',
                        'publishLaserScan.inputs:timeStamp',
                    ),
                    (
                        'readLidarBeams.outputs:horizontalFov',
                        'publishLaserScan.inputs:horizontalFov',
                    ),
                    (
                        'readLidarBeams.outputs:horizontalResolution',
                        'publishLaserScan.inputs:horizontalResolution',
                    ),
                    ('readLidarBeams.outputs:depthRange',
                     'publishLaserScan.inputs:depthRange'),
                    ('readLidarBeams.outputs:rotationRate',
                     'publishLaserScan.inputs:rotationRate'),
                    (
                        'readLidarBeams.outputs:linearDepthData',
                        'publishLaserScan.inputs:linearDepthData',
                    ),
                    (
                        'readLidarBeams.outputs:intensitiesData',
                        'publishLaserScan.inputs:intensitiesData',
                    ),
                    ('readLidarBeams.outputs:numRows',
                     'publishLaserScan.inputs:numRows'),
                    ('readLidarBeams.outputs:numCols',
                     'publishLaserScan.inputs:numCols'),
                    ('readLidarBeams.outputs:azimuthRange',
                     'publishLaserScan.inputs:azimuthRange'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('publishLaserScan.inputs:frameId', 'base_range_sensor_link'),
                    (
                        'publishLaserScan.inputs:topicName',
                        '/scan' if is_ros2 else self.prefix + '/base_scan',
                    ),
                ],
            },
        )
        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath('/lidar_sensor/readLidarBeams'),
            attribute='inputs:lidarPrim',
            target_prim_paths=[self.stage_path +
                               self.prefix + '/base_range_sensor_link/Lidar'],
        )

    def _create_controller_parameter_services(self) -> None:
        self._arm_controller_joints = [
            'arm_lift_joint',
            'arm_flex_joint',
            'arm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
        ]
        self._head_controller_joints = [
            'head_pan_joint',
            'head_tilt_joint',
        ]
        self._base_controller_coordinates = [
            'odom_x',
            'odom_y',
            'odom_t',
        ]
        self._gripper_controller_joints = [
            'hand_motor_joint',
        ]

        self._arm_get_parameters_srv = self.ros2node.create_service(
            GetParameters,
            '/arm_trajectory_controller/get_parameters',
            lambda request, context: self._build_parameters_response(
                request, 'joints', self._arm_controller_joints
            ),
        )
        self._head_get_parameters_srv = self.ros2node.create_service(
            GetParameters,
            '/head_trajectory_controller/get_parameters',
            lambda request, context: self._build_parameters_response(
                request, 'joints', self._head_controller_joints
            ),
        )
        self._base_get_parameters_srv = self.ros2node.create_service(
            GetParameters,
            '/omni_base_controller/get_parameters',
            lambda request, context: self._build_parameters_response(
                request, 'base_coordinates', self._base_controller_coordinates
            ),
        )
        self._gripper_get_parameters_srv = self.ros2node.create_service(
            GetParameters,
            '/gripper_controller/get_parameters',
            lambda request, context: self._build_parameters_response(
                request, 'joints', self._gripper_controller_joints
            ),
        )

    def _build_parameters_response(self, request, valid_name, values):
        response = GetParameters.Response()
        for name in request.names:
            pv = ParameterValue()
            if name == valid_name:
                pv.type = ParameterType.PARAMETER_STRING_ARRAY
                pv.string_array_value = list(values)
            else:
                pv.type = ParameterType.PARAMETER_NOT_SET
            response.values.append(pv)
        return response

    def _bind_physics_material(self, material_path, prim_paths):
        """Bind a USD material for PhysX without the removed BindMaterialExt command."""
        current_stage = stage.get_current_stage()
        material_prim = current_stage.GetPrimAtPath(material_path)
        if not material_prim or not material_prim.IsValid():
            raise RuntimeError('missing physics material: %s' % material_path)
        material = UsdShade.Material(material_prim)

        bound = []
        for prim_path in prim_paths:
            prim = current_stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                raise RuntimeError('missing collision prim: %s' % prim_path)
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
            binding_api.Bind(
                material,
                UsdShade.Tokens.weakerThanDescendants,
                'physics',
            )
            bound.append(prim_path)
        return bound

    def _set_base_root_mass(self):
        # base_footprint is an articulation body without a collider.  Its USD
        # mass=0 makes PhysX substitute a tiny sphere inertia, which gives the
        # mobile base an ill-conditioned root during arm/base motion.  Direct
        # drive can only set state on this root, so give it comparable inertia
        # to base_link instead of leaving a 1 kg body fixed to a 50 kg chassis.
        root_mass = 50.0 if _BASE_DIRECT_DRIVE else 1.0
        root_inertia = (
            Gf.Vec3f(0.4, 0.4, 0.2)
            if _BASE_DIRECT_DRIVE
            else Gf.Vec3f(0.02, 0.02, 0.02)
        )
        root_damping = (
            _BASE_DIRECT_ROOT_DAMPING if _BASE_DIRECT_DRIVE else 2.0)
        root_path = self.stage_path + self.prefix + '/base_footprint'
        root_prim = stage.get_current_stage().GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            raise RuntimeError('missing articulation root: %s' % root_path)
        mass_api = UsdPhysics.MassAPI.Apply(root_prim)
        mass_api.CreateMassAttr().Set(root_mass)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        mass_api.CreateDiagonalInertiaAttr().Set(root_inertia)
        rigid_body_api = PhysxSchema.PhysxRigidBodyAPI(root_prim)
        if not rigid_body_api:
            rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim)
        rigid_body_api.CreateLinearDampingAttr().Set(root_damping)
        rigid_body_api.CreateAngularDampingAttr().Set(root_damping)
        print(
            '[base-physics] base_footprint mass=%.1fkg '
            'inertia=%s damping=%.1f' %
            (root_mass, tuple(root_inertia), root_damping),
            flush=True,
        )

    def _add_planar_root_joint(self):
        if not _BASE_DIRECT_DRIVE:
            return

        # articulation root を毎フレーム直接テレポートすると、内部関節と外部 D6
        # 拘束の解決が衝突して物理が暴れる。kinematic 時は「衝突形状を持たない
        # kinematic anchor」を作り、base_footprint を FixedJoint で結ぶ。移動するのは
        # anchor だけなので、ロボット内部の物理を保ったまま決定論的に台車を拘束できる。
        if _BASE_KINEMATIC_DRIVE:
            current_stage = stage.get_current_stage()
            root_path = self.stage_path + self.prefix + '/base_footprint'
            root_prim = current_stage.GetPrimAtPath(root_path)
            if not root_prim or not root_prim.IsValid():
                raise RuntimeError('missing kinematic base body: %s' % root_path)
            root_transform = UsdGeom.Xformable(
                root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            root_translation = root_transform.ExtractTranslation()
            root_quat = Gf.Transform(root_transform).GetRotation().GetQuat()
            root_imaginary = root_quat.GetImaginary()
            root_yaw = euler_from_quaternion(
                float(root_imaginary[0]), float(root_imaginary[1]),
                float(root_imaginary[2]), float(root_quat.GetReal()))[2]

            anchor_path = self.stage_path + '/hsr_base_kinematic_anchor'
            anchor = UsdGeom.Xform.Define(current_stage, anchor_path)
            anchor_xform = UsdGeom.XformCommonAPI(anchor)
            anchor_xform.SetTranslate(root_translation)
            anchor_xform.SetRotate(
                Gf.Vec3f(0.0, 0.0, math.degrees(root_yaw)),
                UsdGeom.XformCommonAPI.RotationOrderXYZ,
            )
            anchor_body = UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
            anchor_body.CreateKinematicEnabledAttr().Set(True)

            joint_path = self.stage_path + '/hsr_kinematic_base_joint'
            joint = UsdPhysics.FixedJoint.Define(current_stage, joint_path)
            joint.CreateBody0Rel().SetTargets([Sdf.Path(anchor_path)])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            joint.CreateExcludeFromArticulationAttr().Set(True)
            joint.CreateBreakForceAttr().Set(1e20)
            joint.CreateBreakTorqueAttr().Set(1e20)

            self._base_kinematic_anchor_path = anchor_path
            self._base_kinematic_anchor = None
            self._base_brake_drives = {}
            self._planar_frame0 = None
            print(
                '[base-physics] kinematic drive: fixed anchor=%s' % anchor_path,
                flush=True,
            )
            return

        current_stage = stage.get_current_stage()
        root_path = self.stage_path + self.prefix + '/base_footprint'
        root_prim = current_stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            raise RuntimeError('missing planar-joint body: %s' % root_path)

        root_transform = UsdGeom.Xformable(
            root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        root_translation = root_transform.ExtractTranslation()
        root_quat = Gf.Transform(root_transform).GetRotation().GetQuat()
        root_imaginary = root_quat.GetImaginary()
        root_orientation = Gf.Quatf(
            float(root_quat.GetReal()),
            Gf.Vec3f(
                float(root_imaginary[0]),
                float(root_imaginary[1]),
                float(root_imaginary[2]),
            ),
        )

        joint_path = self.stage_path + '/hsr_planar_world_joint'
        joint = UsdPhysics.Joint.Define(current_stage, joint_path)
        joint.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(root_translation[0]),
            float(root_translation[1]),
            float(root_translation[2]),
        ))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(root_orientation)
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        joint.CreateExcludeFromArticulationAttr().Set(True)
        joint.CreateBreakForceAttr().Set(1e20)
        joint.CreateBreakTorqueAttr().Set(1e20)

        # A D6 axis is locked when low > high.  Leave transX/transY/rotZ
        # without LimitAPI so the mobile base keeps its planar degrees of
        # freedom while vertical, roll, and pitch reactions are solved as
        # constraints instead of being cancelled one frame later.
        for axis in ('transZ', 'rotX', 'rotY'):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr().Set(1.0)
            limit.CreateHighAttr().Set(-1.0)

        # 駐車ブレーキ用ドライブ。台車を保持する力を「拘束ソルバー内」で出せる
        # 唯一の経路。stiffness は起動時(物理構築前)に入れておく — 実行中の
        # USD 属性変更は PhysX に伝わらない場合があるため。保持位置は
        # targetPosition を毎ステップ更新して指定する。走行中は現在位置を
        # 目標にし続ければバネ力はほぼ 0 になり、従来の速度制御を妨げない。
        # joint frame0 は spawn 姿勢なので DOF 値 = odom 座標と一致する。
        self._base_brake_drives = {}
        # kinematic drive は停止中も articulation 姿勢を直接保持する。そこでさらに
        # 20万 N/m の位置ドライブを有効にすると、姿勢更新直後の物理ステップで
        # 引き戻しが起きる。kinematic 時は D6 の z/roll/pitch 拘束だけを残す。
        if _BASE_JOINT_BRAKE and not _BASE_KINEMATIC_DRIVE:
            for axis in ('transX', 'transY', 'rotZ'):
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
                drive.CreateTypeAttr().Set('force')
                if axis == 'rotZ':
                    # USD の回転ドライブは度単位。
                    drive.CreateStiffnessAttr().Set(
                        _BASE_BRAKE_ANGULAR_K * math.pi / 180.0)
                    drive.CreateDampingAttr().Set(
                        _BASE_BRAKE_ANGULAR_C * math.pi / 180.0)
                    drive.CreateMaxForceAttr().Set(_BASE_BRAKE_MAX_TORQUE)
                else:
                    drive.CreateStiffnessAttr().Set(_BASE_BRAKE_K)
                    drive.CreateDampingAttr().Set(_BASE_BRAKE_C)
                    drive.CreateMaxForceAttr().Set(_BASE_BRAKE_MAX_FORCE)
                drive.CreateTargetPositionAttr().Set(0.0)
                self._base_brake_drives[axis] = drive
        # ドライブの DOF 値は「この frame0 からの変位」。odom の原点は起動直後の
        # わずかな移動でここからずれるので、その差を後で補正する必要がある
        # (補正しないとサーボの指令が常にずれ、台車が目標に到達できない)。
        self._planar_frame0 = (
            float(root_translation[0]),
            float(root_translation[1]),
            float(euler_from_quaternion(
                float(root_orientation.GetImaginary()[0]),
                float(root_orientation.GetImaginary()[1]),
                float(root_orientation.GetImaginary()[2]),
                float(root_orientation.GetReal()))[2]),
        )
        print(
            '[base-physics] planar joint: free=(x,y,yaw) '
            'locked=(z,roll,pitch) joint_brake=%s frame0=(%.4f, %.4f, %.4f) '
            'k=%.0f d=%.0f'
            % ('on' if self._base_brake_drives else 'off',
               float(root_translation[0]), float(root_translation[1]),
               float(root_translation[2]),
               _BASE_BRAKE_K, _BASE_BRAKE_C),
            flush=True,
        )

    def _tune_articulation_solver(self):
        root_path = self.stage_path + self.prefix
        root_prim = stage.get_current_stage().GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            raise RuntimeError('missing articulation prim: %s' % root_path)
        articulation_api = PhysxSchema.PhysxArticulationAPI(root_prim)
        if not articulation_api:
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(
                root_prim)

        # The imported asset requests 16 velocity iterations.  Isaac Sim 4.5
        # explicitly warns that TGS behavior changed above four iterations;
        # for the offset caster it over-solves alternating tire/steer impulses
        # and produces wheel-speed spikes well beyond the commanded limit.
        articulation_api.CreateSolverVelocityIterationCountAttr().Set(4)
        print(
            '[base-physics] articulation solver velocity iterations=4',
            flush=True,
        )

    def _tune_base_joint_dynamics(self):
        joint_specs = {
            'base_l_drive_wheel_joint': (8.5, 0.015),
            'base_r_drive_wheel_joint': (8.5, 0.015),
            'base_roll_joint': (1.8, 0.05),
        }
        found = set()
        for prim in stage.get_current_stage().Traverse():
            name = prim.GetName()
            if name not in joint_specs:
                continue
            max_velocity, armature = joint_specs[name]
            joint_api = PhysxSchema.PhysxJointAPI(prim)
            if not joint_api:
                joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)
            # Angular USD attributes use degrees while dynamic-control uses
            # radians.  The imported 1191.75 deg/s cap allowed contact impulses
            # to drive wheels to 20.8 rad/s despite the 8.5 rad/s command cap.
            joint_api.CreateMaxJointVelocityAttr().Set(
                math.degrees(max_velocity))
            joint_api.CreateArmatureAttr().Set(armature)
            found.add(name)
            print(
                '[base-joint] %s maxVel=%.3g rad/s armature=%.3g'
                % (name, max_velocity, armature),
                flush=True,
            )
        missing = set(joint_specs) - found
        if missing:
            raise RuntimeError(
                'base joint prims not found: %s' % sorted(missing))

    def _fix_arm_link_inertia(self):
        # hsrb4s.usd was generated from an old test URDF that has arm_flex
        # ixx=7.528.  The current hsrb_description URDF has ixx=0.007528;
        # the 1000x error makes the flex drive saturate and oscillate.
        link_path = self.stage_path + self.prefix + '/arm_flex_link'
        link_prim = stage.get_current_stage().GetPrimAtPath(link_path)
        if not link_prim or not link_prim.IsValid():
            raise RuntimeError('missing arm link prim: %s' % link_path)
        mass_api = UsdPhysics.MassAPI.Apply(link_prim)
        mass_api.CreateDiagonalInertiaAttr().Set(
            Gf.Vec3f(0.007528, 0.007102, 0.001552))
        mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0))
        print(
            '[arm-physics] arm_flex_link inertia='
            '(0.007528,0.007102,0.001552)kg*m^2',
            flush=True,
        )

    def _tune_arm_joint_dynamics(self):
        # The imported model has zero reflected motor inertia on every joint.
        # Once arm_flex's 1000x inertia typo is corrected, the light wrist
        # bodies otherwise respond to a one-step drive impulse too quickly for
        # the 60 Hz controller.  Small armatures keep the inertia ratios
        # well-conditioned without changing link mass or gravity torque.
        armatures = {
            'arm_flex_joint': 0.05,
            'arm_roll_joint': 0.02,
            'wrist_flex_joint': 0.01,
            'wrist_roll_joint': 0.01,
        }
        found = set()
        for prim in stage.get_current_stage().Traverse():
            name = prim.GetName()
            if name not in armatures:
                continue
            joint_api = PhysxSchema.PhysxJointAPI(prim)
            if not joint_api:
                joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)
            joint_api.CreateArmatureAttr().Set(armatures[name])
            found.add(name)
            print(
                '[arm-joint] %s armature=%.3g'
                % (name, armatures[name]),
                flush=True,
            )
        missing = set(armatures) - found
        if missing:
            raise RuntimeError(
                'arm joint prims not found: %s' % sorted(missing))

    def _tune_arm_drives(self):
        # The imported USD uses generic gains (k=1e7, d=1e5) and force limits
        # of 18 kN / 6 kNm.  Angular USD gains are torque/degree, while the
        # controller and values below use radians.  Convert explicitly;
        # assigning the per-radian values directly made the effective gains
        # 57.3x too high and turned the position drive into a force limiter.
        drive_specs = {
            'arm_lift_joint': ('linear', 8000.0, 1000.0, 300.0),
            'torso_lift_joint': ('linear', 8000.0, 1000.0, 300.0),
            # k=500 だと腕を水平近くまで伸ばしたとき重力で 0.02〜0.04 rad
            # (先端で 2〜3cm) 垂れ、目標許容 0.03 を超えて whole_body 動作が
            # 「目標未達」で中断していた。剛性を上げてたわみを 1/2 以下にする。
            # 台車は平面ジョイントのドライブで固定されるので、反力が増えても
            # 車体が揺すられることはない。
            'arm_flex_joint': ('angular', 1200.0, 200.0, _ARM_FLEX_MAX_FORCE),
            'arm_roll_joint': ('angular', 200.0, 30.0, 100.0),
            'wrist_flex_joint': ('angular', 200.0, 30.0, 100.0),
            'wrist_roll_joint': ('angular', 100.0, 20.0, 100.0),
        }
        found = set()
        for prim in stage.get_current_stage().Traverse():
            name = prim.GetName()
            if name not in drive_specs:
                continue
            drive_type, stiffness_rad, damping_rad, max_force = (
                drive_specs[name])
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if not drive:
                raise RuntimeError(
                    'missing %s drive for %s' % (drive_type, prim.GetPath()))
            gain_scale = math.pi / 180.0 if drive_type == 'angular' else 1.0
            stiffness = stiffness_rad * gain_scale
            damping = damping_rad * gain_scale
            drive.GetStiffnessAttr().Set(stiffness)
            drive.GetDampingAttr().Set(damping)
            drive.GetMaxForceAttr().Set(max_force)
            drive.GetTargetVelocityAttr().Set(0.0)
            drive.GetTypeAttr().Set('force')
            found.add(name)
            print(
                '[arm-drive] %s k=%.3g/rad d=%.3g/rad '
                '(USD k=%.3g d=%.3g) maxF=%.1f'
                % (
                    name,
                    stiffness_rad,
                    damping_rad,
                    stiffness,
                    damping,
                    max_force,
                ),
                flush=True,
            )
        missing = set(drive_specs) - found
        if missing:
            raise RuntimeError(
                'arm drive prims not found: %s' % sorted(missing))

    def set_base_joint_and_material(self) -> None:
        self._set_base_root_mass()
        self._add_planar_root_joint()
        self._tune_articulation_solver()
        self._tune_base_joint_dynamics()
        self._fix_arm_link_inertia()
        self._tune_arm_joint_dynamics()
        self._tune_arm_drives()

        caster_material = PhysicsMaterial(
            prim_path='/Caster',
            static_friction=0.0,
            dynamic_friction=0.0,
        )
        caster_api = PhysxSchema.PhysxMaterialAPI.Apply(
            stage.get_current_stage().GetPrimAtPath('/Caster'))
        caster_api.CreateFrictionCombineModeAttr().Set('min')
        caster_paths = [
            self.stage_path + self.prefix + l
            for l in (
                '/base_l_passive_wheel_z_link/collisions',
                '/base_r_passive_wheel_z_link/collisions',
            )
        ]
        self._bind_physics_material('/Caster', caster_paths)

        # Direct mode is an ideal planar actuator for MoveIt whole-body tests.
        # Removing longitudinal tire friction prevents the unactuated
        # twin-caster joints from feeding steering impulses back into the
        # 70 kg chassis.  Normal contact remains active, so the base is still
        # supported by the floor and participates in vertical collisions.
        tire_static_friction = 0.0 if _BASE_DIRECT_DRIVE else 1.0
        tire_dynamic_friction = 0.0 if _BASE_DIRECT_DRIVE else 0.8
        tire_material = PhysicsMaterial(
            prim_path='/Tire',
            static_friction=tire_static_friction,
            dynamic_friction=tire_dynamic_friction,
        )
        tire_api = PhysxSchema.PhysxMaterialAPI.Apply(
            stage.get_current_stage().GetPrimAtPath('/Tire'))
        tire_api.CreateFrictionCombineModeAttr().Set(
            'min' if _BASE_DIRECT_DRIVE else 'max')
        tire_paths = [
            self.stage_path + self.prefix + l
            for l in (
                '/base_l_drive_wheel_link/collisions',
                '/base_r_drive_wheel_link/collisions',
            )
        ]
        self._bind_physics_material('/Tire', tire_paths)
        print(
            '[base-physics] mode=%s caster friction=0 tire friction=(%.1f,%.1f)'
            % (
                'direct' if _BASE_DIRECT_DRIVE else 'wheel',
                tire_static_friction,
                tire_dynamic_friction,
            ),
            flush=True,
        )

        # グリッパ指に高摩擦マテリアルを付ける。既定摩擦だと軽い缶が握っても滑って
        # 抜ける(実測: 0.4cm持ち上げて落ちる)。指の衝突プリム(hand_*/.../collisions)を
        # 自動探索してバインドする(リンク名の取り違え回避)。
        finger_material = PhysicsMaterial(
            prim_path='/GripperFinger',
            static_friction=30.0,
            dynamic_friction=30.0,
        )
        # 摩擦の合成モードを max にする。既定(average)だと缶側の低い摩擦と平均されて
        # 弱まるが、max なら指の高摩擦(8.0)が支配して滑りにくくなる。
        try:
            _fm_prim = stage.get_current_stage().GetPrimAtPath('/GripperFinger')
            _fm_api = PhysxSchema.PhysxMaterialAPI.Apply(_fm_prim)
            _fm_api.CreateFrictionCombineModeAttr().Set('max')
            print('[grip-fric] frictionCombineMode=max', flush=True)
        except Exception as _e:
            print('[grip-fric] combine-mode err %r' % _e, flush=True)
        _root = self.stage_path + self.prefix
        _st = stage.get_current_stage()
        _finger_paths = []
        for _p in _st.Traverse():
            _ps = str(_p.GetPath())
            if _ps.startswith(_root) and 'hand_' in _ps and _ps.endswith('/collisions'):
                _finger_paths.append(_ps)
        _bound = self._bind_physics_material(
            '/GripperFinger', _finger_paths)
        print(
            '[grip-fric] bound finger material to %d prims: %s' % (len(_bound), _bound), flush=True
        )

        # 指コライダーが動的物体(缶)と接触判定を起こさない問題への対策(実測+GUIで確認:
        # 手のひらは衝突するが指リンクだけ缶を貫通する)。指の collider は convexHull だが、
        # convexHull は cook(凸包の生成計算)に失敗すると enabled のままでも実体の無い
        # 当たり判定になり、見た目だけ残って貫通する(手のひらは cook 成功・指は薄mesh で
        # 失敗、の非対称で説明可)。cook 不要の boundingCube(箱)に変えて確実に実体を作る。
        try:
            from omni.physx.scripts import utils as _pxutils

            _finger_col_prims = [
                self.stage_path + self.prefix + '/hand_l_spring_proximal_link/collisions',
                self.stage_path + self.prefix + '/hand_l_distal_link/collisions',
                self.stage_path + self.prefix + '/hand_r_spring_proximal_link/collisions',
                self.stage_path + self.prefix + '/hand_r_distal_link/collisions',
            ]
            for _cp in _finger_col_prims:
                _cpr = _st.GetPrimAtPath(_cp)
                if not _cpr or not _cpr.IsValid():
                    print('[grip-col] missing %s' % _cp, flush=True)
                    continue
                _pxutils.setCollider(_cpr, approximationShape='convexHull')
                print('[grip-col] convexHull collider (薄いまま) %s' %
                      _cp, flush=True)
        except Exception as _e:
            print('[grip-col] error: %r' % _e, flush=True)

        # グリッパ指関節の駆動を「弱く・ゆっくり」にする。
        # URDF Importer 既定の指ドライブは stiffness(ばね定数)が高く、閉じ指令
        # (position=0)へ一気に駆動するため、軽い自由物体(缶)を弾き飛ばす(実測)。
        # stiffness を下げて握る力を弱め、damping(粘性=動きへの抵抗)を上げて
        # ゆっくり閉じさせ、maxForce(=トルク上限)を絞ることで、「物体に触れたら
        # 弱い力で握って止まる」コンプライアント(柔らかい)な閉じにする。
        # ※ 値は実機合わせ。弾く→さらに下げる / 持ち上げで滑る→少し上げる で調整。
        # 衝突が効くようになったので、握り力を上げて缶を押し付ける(摩擦で保持するため)。
        # 旧設定 (k=8, d=28) は時定数 d/k=3.5s で閉じ切りに ~16 秒かかっていた。
        # 柔らかさ(コンプライアンス)は保ちつつ時定数を 0.5s に短縮し、
        # 開閉を実用速度 (~2 秒) にする。物体を弾くようなら k を下げる。
        FINGER_STIFFNESS = 20.0
        FINGER_DAMPING = 10.0
        FINGER_MAX_FORCE = 10.0
        finger_joint_paths = [
            self.stage_path + self.prefix + '/hand_palm_link/hand_l_proximal_joint',
            self.stage_path + self.prefix + '/hand_l_mimic_distal_link/hand_l_distal_joint',
            self.stage_path + self.prefix + '/hand_palm_link/hand_r_proximal_joint',
            self.stage_path + self.prefix + '/hand_r_mimic_distal_link/hand_r_distal_joint',
        ]
        for _fp in finger_joint_paths:
            _fd = UsdPhysics.DriveAPI.Get(
                stage.get_current_stage().GetPrimAtPath(_fp), 'angular')
            if not _fd:
                print('[grip-drive] DriveAPI not found: %s' % _fp, flush=True)
                continue
            _fd.GetStiffnessAttr().Set(FINGER_STIFFNESS)
            _fd.GetDampingAttr().Set(FINGER_DAMPING)
            _fd.GetMaxForceAttr().Set(FINGER_MAX_FORCE)
            print(
                '[grip-drive] tuned %s (k=%.1f d=%.1f maxF=%.1f)'
                % (_fp, FINGER_STIFFNESS, FINGER_DAMPING, FINGER_MAX_FORCE),
                flush=True,
            )

        left_passive1_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_l_passive_wheel_x_frame/base_l_passive_wheel_y_frame_joint'
            ),
            'angular',
        )
        # 転がり軸: damping=0 だと止まらなくなるので小さな粘性摩擦を入れる
        # (詳細は _BASE_PASSIVE_WHEEL_DAMPING の定義箇所のコメント)。
        left_passive1_drive.GetDampingAttr().Set(_BASE_PASSIVE_WHEEL_DAMPING)
        left_passive1_drive.GetStiffnessAttr().Set(0)

        left_passive2_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_l_passive_wheel_y_frame/base_l_passive_wheel_z_joint'
            ),
            'angular',
        )
        # 旋回軸: 実測で正しく減衰して止まるので無摩擦のままにする
        left_passive2_drive.GetDampingAttr().Set(0)
        left_passive2_drive.GetStiffnessAttr().Set(0)

        right_passive1_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_r_passive_wheel_x_frame/base_r_passive_wheel_y_frame_joint'
            ),
            'angular',
        )
        # 転がり軸: damping=0 だと止まらなくなるので小さな粘性摩擦を入れる
        # (詳細は _BASE_PASSIVE_WHEEL_DAMPING の定義箇所のコメント)。
        right_passive1_drive.GetDampingAttr().Set(_BASE_PASSIVE_WHEEL_DAMPING)
        right_passive1_drive.GetStiffnessAttr().Set(0)

        right_passive2_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path
                + self.prefix
                + '/base_r_passive_wheel_y_frame/base_r_passive_wheel_z_joint'
            ),
            'angular',
        )
        # 旋回軸: 実測で正しく減衰して止まるので無摩擦のままにする
        right_passive2_drive.GetDampingAttr().Set(0)
        right_passive2_drive.GetStiffnessAttr().Set(0)

        left_wheel_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path + self.prefix + '/base_roll_link/base_l_drive_wheel_joint'
            ),
            'angular',
        )
        right_wheel_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path + self.prefix + '/base_roll_link/base_r_drive_wheel_joint'
            ),
            'angular',
        )
        roll_drive = UsdPhysics.DriveAPI.Get(
            stage.get_current_stage().GetPrimAtPath(
                self.stage_path + self.prefix + '/base_link/base_roll_joint'
            ),
            'angular',
        )

        # Imported defaults make lateral steering effectively bang-bang.
        # Respect the URDF effort limits so velocity error cannot kick the
        # chassis through the high-friction drive wheels.
        wheel_damping = (
            _BASE_DIRECT_JOINT_DAMPING
            if _BASE_DIRECT_DRIVE
            else _BASE_WHEEL_DRIVE_DAMPING
        )
        steer_damping = (
            _BASE_DIRECT_JOINT_DAMPING
            if _BASE_DIRECT_DRIVE
            else _BASE_STEER_DRIVE_DAMPING
        )
        left_wheel_drive.GetDampingAttr().Set(wheel_damping)
        right_wheel_drive.GetDampingAttr().Set(wheel_damping)
        roll_drive.GetDampingAttr().Set(steer_damping)
        wheel_max_force = (
            _BASE_DIRECT_JOINT_MAX_FORCE
            if _BASE_DIRECT_DRIVE
            else _BASE_WHEEL_DRIVE_MAX_FORCE
        )
        steer_max_force = (
            _BASE_DIRECT_JOINT_MAX_FORCE
            if _BASE_DIRECT_DRIVE
            else _BASE_STEER_DRIVE_MAX_FORCE
        )
        left_wheel_drive.GetMaxForceAttr().Set(wheel_max_force)
        right_wheel_drive.GetMaxForceAttr().Set(wheel_max_force)
        roll_drive.GetMaxForceAttr().Set(steer_max_force)

        left_wheel_drive.GetStiffnessAttr().Set(0)
        right_wheel_drive.GetStiffnessAttr().Set(0)
        roll_drive.GetStiffnessAttr().Set(0)
        print(
            '[base-drive] damping=(wheel:%.3g, steer:%.3g) '
            'maxForce=(wheel:%.3g, steer:%.3g)'
            % (
                wheel_damping,
                steer_damping,
                wheel_max_force,
                steer_max_force,
            ),
            flush=True,
        )

    def on_cmd_vel(self, msg):
        if self.simulation_context is not None:
            self.cmd_vel_msg = msg
            self.last_cmd_vel_time = self.simulation_context.current_time

    def on_laserscan_odom(self, msg):
        posestamped = PoseStamped()
        posestamped.header.stamp = msg.header.stamp
        posestamped.header.frame_id = 'world'
        posestamped.pose = msg.pose.pose
        self.on_laserscan_pose(posestamped)

    def on_laserscan_pose(self, msg):
        if not is_ros2:
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id = 'world'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose = msg.pose
            odom.pose.covariance = [
                0.001,
                0,
                0,
                0,
                0,
                0,
                0,
                0.001,
                0,
                0,
                0,
                0,
                0,
                0,
                100000.0,
                0,
                0,
                0,
                0,
                0,
                0,
                100000.0,
                0,
                0,
                0,
                0,
                0,
                0,
                100000.0,
                0,
                0,
                0,
                0,
                0,
                0,
                1000.0,
            ]
            self.laser_odom_pub.publish(odom)

        q = msg.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)[2]
        # 【無効化】base の odom をレーザ pose で上書きすると、scan が不良(壁を捉え
        # られず最大レンジばかり)なときにこの pose がドリフトし、台車の軌道P制御が
        # それを追いかけて指令ゼロでもゆっくり回り続ける原因になる。
        # Sim では車輪オドメトリ(WheelOdometry.integrate)が正確なので base odom は
        # それに任せ、ここでのレーザ上書きは行わない。地図との整合は別の
        # localization(map->odom)が担当する想定。
        # self.odometry_estimator.set_pose(
        #     msg.pose.position.x, msg.pose.position.y, yaw)

    def publish_joint_states(self, dt):
        js = JointState()
        js.header.stamp = self.get_ros_time(
            self.simulation_context.current_time)
        js.name = list(self._joints.keys())
        previous_positions = getattr(
            self, '_joint_state_previous_positions', {})
        filtered_velocities = getattr(
            self, '_joint_state_filtered_velocities', {})
        for n in js.name:
            (joint, joint_type, inv) = self._joints[n]
            st = self.dc.get_dof_state(joint, _dynamic_control.STATE_ALL)
            position = st.pos
            if joint_type == _dynamic_control.JOINT_PRISMATIC:
                position *= get_stage_units()
            if inv:
                position = -position
                st.effort = -st.effort
            previous_position = previous_positions.get(n)
            previous_velocity = filtered_velocities.get(n, 0.0)
            if previous_position is None or dt <= 1e-9:
                velocity = 0.0
            else:
                position_delta = position - previous_position
                if joint_type == _dynamic_control.JOINT_REVOLUTE:
                    position_delta = math.atan2(
                        math.sin(position_delta),
                        math.cos(position_delta),
                    )
                raw_velocity = position_delta / dt
                alpha = min(1.0, dt / (0.05 + dt))
                velocity = previous_velocity + alpha * (
                    raw_velocity - previous_velocity)
            previous_positions[n] = position
            filtered_velocities[n] = velocity
            js.position.append(position)
            js.velocity.append(velocity)
            js.effort.append(st.effort * 1000.0)
        self._joint_state_previous_positions = previous_positions
        self._joint_state_filtered_velocities = filtered_velocities
        if not is_ros2:
            js.name = js.name + ['odom_x', 'odom_y', 'odom_t']
            js.position.extend([
                self.odometry_estimator.pose.x,
                self.odometry_estimator.pose.y,
                self.odometry_estimator.pose.ang,
            ])
            js.velocity.extend([0, 0, 0])
            js.effort.extend([0, 0, 0])
        self.joint_state_pub.publish(js)

    def _object_body_paths(self):
        # ロボット以外の剛体(=掴める物体)プリムのパス一覧を一度だけ集めてキャッシュ。
        if self._obj_paths_cache is not None:
            return self._obj_paths_cache
        paths = []
        try:
            _st = stage.get_current_stage()
            _root = self.stage_path + self.prefix
            for _p in _st.Traverse():
                _ps = str(_p.GetPath())
                if _ps.startswith(_root):
                    continue  # ロボット自身は除く
                if _ps == getattr(self, '_base_kinematic_anchor_path', None):
                    continue  # 衝突のない台車制御用anchorは把持対象ではない
                if _p.HasAPI(UsdPhysics.RigidBodyAPI):
                    paths.append(_ps)
        except Exception as _e:
            print('[graspA] object scan err %r' % _e, flush=True)
        self._obj_paths_cache = paths
        print('[graspA] graspable bodies: %s' % paths, flush=True)
        return paths

    def _set_grasp_object_collision(self, body_path, enabled):
        """把持中の物体の衝突を一時的に切る/戻す。

        案A(アタッチ把持)は物体を毎ステップ手の位置へテレポートして保持する。物体の
        衝突(recol で有効化)が残っていると、テレポート先で指と毎ステップ押し合い、
        移動時に発散してロボットごと吹き飛ぶ。把持中は衝突を切り、離したら戻す。
        """
        try:
            import omni.usd
            from pxr import Usd, UsdPhysics

            _st = omni.usd.get_context().get_stage()
            _root = _st.GetPrimAtPath(body_path)
            if not _root or not _root.IsValid():
                return
            for _p in Usd.PrimRange(_root):
                if _p.HasAPI(UsdPhysics.CollisionAPI):
                    _a = _p.GetAttribute('physics:collisionEnabled')
                    if not _a:
                        _a = UsdPhysics.CollisionAPI(
                            _p).CreateCollisionEnabledAttr()
                    _a.Set(bool(enabled))
        except Exception as _e:
            print('[graspA] collision toggle err %r' % _e, flush=True)

    def _grasp_attach_update(self):
        # 案A: 指の衝突が効かない問題を迂回して把持を再現する。
        # グリッパが閉じていて把持中心の近くに物体があれば、その物体をグリッパに
        # 追従させる(=掴む)。グリッパが開いたら追従を止める(=離す)。
        if not getattr(self, '_joints', None) or 'hand_motor_joint' not in self._joints:
            return
        try:
            _hm = self.dc.get_dof_state(
                self._joints['hand_motor_joint'][0], _dynamic_control.STATE_POS
            ).pos
        except Exception:
            return
        if self._palm_body is None:
            self._palm_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_palm_link'
            )
        if getattr(self, '_lfinger_body', None) is None:
            self._lfinger_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_l_distal_link'
            )
            self._rfinger_body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/hand_r_distal_link'
            )
        if not self._palm_body or not self._lfinger_body or not self._rfinger_body:
            return
        _palm = self.dc.get_rigid_body_pose(self._palm_body)
        _pp = (_palm.p.x, _palm.p.y, _palm.p.z)
        _pq = (_palm.r.x, _palm.r.y, _palm.r.z, _palm.r.w)
        # 把持中心 = 左右の指先(distal)の中点。物体が来るべき場所そのもの。
        _lf = self.dc.get_rigid_body_pose(self._lfinger_body).p
        _rf = self.dc.get_rigid_body_pose(self._rfinger_body).p
        _gc = ((_lf.x + _rf.x) / 2.0, (_lf.y + _rf.y) /
               2.0, (_lf.z + _rf.z) / 2.0)

        CLOSE_T, OPEN_T, GRASP_DIST = 0.5, 0.6, 0.15
        if self._grasp_obj is None:
            if _hm < CLOSE_T:  # 閉じている/閉じ動作中
                _best = None
                _bestd = GRASP_DIST
                _mind = 999.0
                for _bp in self._object_body_paths():
                    _h = self.dc.get_rigid_body(_bp)
                    if not _h:
                        continue
                    _op = self.dc.get_rigid_body_pose(_h)
                    _d = math.sqrt(
                        (_gc[0] - _op.p.x) ** 2 + (_gc[1] -
                                                   _op.p.y) ** 2 + (_gc[2] - _op.p.z) ** 2
                    )
                    _mind = min(_mind, _d)
                    if _d < _bestd:
                        _bestd = _d
                        _best = (_bp, _h, _op)
                if _mind < 0.4 and not getattr(self, '_grasp_dbg_done', False):
                    print(
                        '[graspA] try: gc=(%.3f,%.3f,%.3f) nearest_obj_dist=%.3f (閾値%.2f)'
                        % (_gc[0], _gc[1], _gc[2], _mind, GRASP_DIST),
                        flush=True,
                    )
                    self._grasp_dbg_done = True
                if _best is not None:
                    _bp, _h, _op = _best
                    _rel_p = _q_rot(
                        _q_conj(_pq), (_op.p.x -
                                       _pp[0], _op.p.y - _pp[1], _op.p.z - _pp[2])
                    )
                    _rel_q = _q_mul(
                        _q_conj(_pq), (_op.r.x, _op.r.y, _op.r.z, _op.r.w))
                    self._grasp_obj = {
                        'h': _h, 'rel_p': _rel_p, 'rel_q': _rel_q, 'path': _bp}
                    # 把持中はテレポート保持なので衝突を切る(指との押し合いで発散しない)。
                    self._set_grasp_object_collision(_bp, False)
                    print('[graspA] 掴んだ: %s (dist=%.3f)' %
                          (_bp, _bestd), flush=True)
        else:
            if _hm > OPEN_T:  # 開いた → 離す
                try:
                    self.dc.set_rigid_body_linear_velocity(
                        self._grasp_obj['h'], (0.0, 0.0, 0.0))
                    self.dc.set_rigid_body_angular_velocity(
                        self._grasp_obj['h'], (0.0, 0.0, 0.0))
                except Exception:
                    pass
                # 離したら衝突を戻す(机に乗る/他物体と当たる)。
                self._set_grasp_object_collision(self._grasp_obj['path'], True)
                print('[graspA] 離した: %s' % self._grasp_obj['path'], flush=True)
                self._grasp_obj = None
                self._grasp_dbg_done = False  # 次の閉じで再びデバッグ出力
            else:  # 掴んだまま → palm に追従
                _g = self._grasp_obj
                _wp = _q_rot(_pq, _g['rel_p'])
                _wp = (_pp[0] + _wp[0], _pp[1] + _wp[1], _pp[2] + _wp[2])
                _wq = _q_mul(_pq, _g['rel_q'])
                _t = _dynamic_control.Transform()
                _t.p = _wp
                _t.r = _wq
                self.dc.set_rigid_body_pose(_g['h'], _t)
                self.dc.set_rigid_body_linear_velocity(
                    _g['h'], (0.0, 0.0, 0.0))
                self.dc.set_rigid_body_angular_velocity(
                    _g['h'], (0.0, 0.0, 0.0))

    def onsimulationstart(self, simulation_context):
        self.simulation_context = simulation_context
        self.prev_time = self.simulation_context.current_time
        self._joint_state_previous_positions = {}
        self._joint_state_filtered_velocities = {}
        # 案A(アタッチ把持)用の状態
        self._grasp_obj = None  # 掴んでいる物体 {h, rel_p, rel_q, path}
        self._palm_body = None  # hand_palm_link の剛体ハンドル(キャッシュ)
        self._obj_paths_cache = None  # 物体(剛体)プリムのパス一覧(キャッシュ)

    def _reset_base_pose_integral(self):
        self._base_pose_error_integral = [0.0, 0.0, 0.0]

    def _dump_object_poses(self):
        """掴める物体の実際の位置 (world と odom) を一度だけ出力する。"""
        ref = getattr(self, '_gt_ref', None)
        print('[objects] 物体の実位置 (落下・転がり後):', flush=True)
        for path in self._object_body_paths():
            try:
                handle = self.dc.get_rigid_body(path)
                if not handle:
                    continue
                pose = self.dc.get_rigid_body_pose(handle)
            except Exception:
                continue
            name = path.strip('/').split('/')[0]
            line = ('[objects]   %-42s world=(%.3f, %.3f, %.3f)'
                    % (name, pose.p.x, pose.p.y, pose.p.z))
            if ref is not None:
                rwx, rwy, _rwyaw, rox, roy, _royaw = ref
                line += ('  odom=(%.3f, %.3f)'
                         % (rox + (pose.p.x - rwx), roy + (pose.p.y - rwy)))
            print(line, flush=True)

    def _dump_physics_report(self):
        """台車挙動の解析用に、シミュレータ側の物理パラメータを一括で出力する。

        質量・慣性・ソルバー設定・関節ドライブ・摩擦が分かると、台車が腕の反力で
        どれだけ動くかが数値で説明できる (BASE_PHYSICS_REPORT=0 で抑制)。
        """
        def _p(line):
            print('[phys] ' + line, flush=True)

        _p('================ physics report ================')
        try:
            _p('physics_dt=%.6f s (%.1f Hz)  rendering_dt=%.6f s'
               % (self.simulation_context.get_physics_dt(),
                  1.0 / max(1e-9, self.simulation_context.get_physics_dt()),
                  self.simulation_context.get_rendering_dt()))
        except Exception as exc:
            _p('timestep 取得失敗: %s' % exc)

        try:
            scene_prim = None
            for prim in stage.get_current_stage().Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    scene_prim = prim
                    break
            if scene_prim is not None:
                usd_scene = UsdPhysics.Scene(scene_prim)
                _p('scene=%s gravity_dir=%s magnitude=%s'
                   % (scene_prim.GetPath(),
                      usd_scene.GetGravityDirectionAttr().Get(),
                      usd_scene.GetGravityMagnitudeAttr().Get()))
                px_scene = PhysxSchema.PhysxSceneAPI(scene_prim)
                for attr in (
                    'GetTimeStepsPerSecondAttr',
                    'GetSolverTypeAttr',
                    'GetEnableGPUDynamicsAttr',
                    'GetEnableStabilizationAttr',
                    'GetBounceThresholdAttr',
                    'GetFrictionOffsetThresholdAttr',
                    'GetMinPositionIterationCountAttr',
                    'GetMaxPositionIterationCountAttr',
                    'GetMinVelocityIterationCountAttr',
                    'GetMaxVelocityIterationCountAttr',
                ):
                    try:
                        a = getattr(px_scene, attr)()
                        _p('  scene.%s = %s' % (a.GetName(), a.Get()))
                    except Exception:
                        pass
        except Exception as exc:
            _p('scene 取得失敗: %s' % exc)

        # --- 質量・慣性 (台車が反力でどれだけ動くかを決める最重要値) ---
        try:
            names = list(self.robots.body_names)
            masses = self.robots.get_body_masses()[0]
            total = float(sum(float(m) for m in masses))
            _p('articulation=%s links=%d total_mass=%.3f kg'
               % (self.robots.prim_paths[0], len(names), total))
            inertias = None
            try:
                inertias = self.robots.get_body_inertias()[0]
            except Exception:
                pass
            for i, name in enumerate(names):
                mass = float(masses[i])
                if mass <= 0.0:
                    continue
                extra = ''
                if inertias is not None:
                    iv = [float(v) for v in inertias[i]]
                    if len(iv) == 9:
                        extra = ' I=(%.4f, %.4f, %.4f)' % (iv[0], iv[4], iv[8])
                _p('  link %-28s m=%8.3f kg%s' % (name, mass, extra))
        except Exception as exc:
            _p('質量取得失敗: %s' % exc)

        # --- 関節ドライブ (腕の駆動力 = 台車を押す力の源) ---
        try:
            for name, (ptr, _jt, _inv) in sorted(self._joints.items()):
                props = self.dc.get_dof_properties(ptr)
                _p('  dof %-28s k=%-10.6g d=%-10.6g maxEffort=%-10.6g '
                   'maxVel=%-8.6g limits=[%.3f, %.3f]'
                   % (name, props.stiffness, props.damping,
                      props.max_effort, props.max_velocity,
                      props.lower, props.upper))
        except Exception as exc:
            _p('dof 取得失敗: %s' % exc)

        # --- 台車設定値 (環境変数で調整しているもの) ---
        _p('base: direct_drive=%s kinematic_drive=%s brake=%s gt_odom=%s'
           % (_BASE_DIRECT_DRIVE, _BASE_KINEMATIC_DRIVE,
              _BASE_BRAKE_ENABLED, _GT_ODOM))
        _p('base: traj P=%.3f I=%.3f D=%.3f deadband=%.4f'
           % (_BASE_TRAJ_P_GAIN, _BASE_TRAJ_I_GAIN, _BASE_TRAJ_D_GAIN,
              _BASE_CONTROL_DEADBAND))
        _p('base: cmd_tau=%.3f accel_lin=%.3f accel_ang=%.3f root_damping=%.3f'
           % (_BASE_CMD_TAU, _BASE_LINEAR_ACCEL_LIMIT,
              _BASE_ANGULAR_ACCEL_LIMIT, _BASE_DIRECT_ROOT_DAMPING))
        _p('base: joint_drive k=%.0f d=%.0f (ang k=%.0f d=%.0f) '
           'setpoint_lag=%.3fm'
           % (_BASE_BRAKE_K, _BASE_BRAKE_C, _BASE_BRAKE_ANGULAR_K,
              _BASE_BRAKE_ANGULAR_C, _BASE_SETPOINT_MAX_LAG))
        _p('base: brake_engage=(%.3f m, %.3f rad) creep=(%.3f m/s, %.3f rad/s)'
           % (_BASE_BRAKE_ENGAGE_LINEAR, _BASE_BRAKE_ENGAGE_ANGULAR,
              _BASE_BRAKE_CREEP_SPEED, _BASE_BRAKE_CREEP_ANGULAR))
        if _BASE_KINEMATIC_DRIVE:
            _p('base: kinematic_collision=%s radius=%.3fm height=%.3fm '
               'margin=%.3fm'
               % (_BASE_KINEMATIC_COLLISION, _BASE_COLLISION_RADIUS,
                  _BASE_COLLISION_HEIGHT, _BASE_COLLISION_MARGIN))
            _p('base: kinematic_anchor_mode=%s'
               % (_BASE_KINEMATIC_ANCHOR_MODE,))
        _p('===============================================')

    def _apply_base_brake(self, target, dt):
        """駐車ブレーキ: 停止すべき間、台車を保持位置へバネで固定する。

        直接駆動モードはタイヤ摩擦が 0 なので、速度をゼロに書くだけでは 1 物理
        ステップ内に生じる腕の反力を受け止められず毎ステップ数 mm ずつ流される
        (実測: 腕を振ると最大 1055mm)。平面ジョイントのドライブで保持すると
        拘束ソルバーの中で反力を受け止められ、3.9mm に収まる。移動指令中は
        解除するので衝突応答は従来どおり。
        """
        pin = getattr(self, '_base_brake_pin', None)
        if pin is None:
            # 掛け始めは現在位置から。いきなり目標へ飛ばすと段差になるため。
            pin = (
                self.odometry_estimator.pose.x,
                self.odometry_estimator.pose.y,
                self.odometry_estimator.pose.ang,
            )
            if _BASE_DIAG:
                print(
                    '[base-brake] 保持開始 (%.4f, %.4f, %.4f)' % pin,
                    flush=True,
                )
        # 残った誤差はゆっくり詰める (瞬間移動させない)。
        if dt > 0.0 and _BASE_BRAKE_CREEP_SPEED > 0.0:
            ex = target[0] - pin[0]
            ey = target[1] - pin[1]
            er = math.atan2(
                math.sin(target[2] - pin[2]), math.cos(target[2] - pin[2]))
            max_lin = _BASE_BRAKE_CREEP_SPEED * dt
            dist = math.hypot(ex, ey)
            if dist > max_lin:
                scale = max_lin / dist
                ex *= scale
                ey *= scale
            max_ang = _BASE_BRAKE_CREEP_ANGULAR * dt
            er = max(-max_ang, min(max_ang, er))
            pin = (pin[0] + ex, pin[1] + ey, pin[2] + er)
        self._base_brake_pin = pin
        self._base_brake_engaged = True

        # 保持の実体はこれ。平面ジョイントのドライブ(バネ・ダンパ)を保持位置へ
        # 向けると、腕の反力を拘束ソルバーの中で受け止められる。
        # (Dynamic Control の外力印加も試したが、2699N 掛けても台車が止まらず
        #  効果ゼロだったため使わない。速度の書き戻しも隣接リンクの慣性に
        #  打ち消される。実際に効くのはこのドライブだけ。)
        self._set_joint_brake_target(pin)

        # 速度ゼロの書き戻しは単独では効かないが、ドライブの補助にはなる。
        try:
            self.robots.set_velocities(np.zeros((1, 6), dtype=np.float32))
        except Exception as exc:
            self._warn_once('brake-vel', 'ブレーキ速度ゼロ化に失敗: %s' % exc)

        if _BASE_DIAG:
            self._base_brake_diag_count = (
                getattr(self, '_base_brake_diag_count', 0) + 1)
            if self._base_brake_diag_count % 30 == 1:
                pose = self.odometry_estimator.pose
                print(
                    '[base-diag] 保持位置=(%.4f, %.4f, %.4f) 実位置=(%.4f,'
                    ' %.4f, %.4f) ずれ=(%.4f, %.4f, %.4f)'
                    % (pin[0], pin[1], pin[2], pose.x, pose.y, pose.ang,
                       pin[0] - pose.x, pin[1] - pose.y,
                       math.atan2(math.sin(pin[2] - pose.ang),
                                  math.cos(pin[2] - pose.ang))),
                    flush=True,
                )

    def _advance_base_setpoint(self, cmd, dt):
        """指令速度を積分して台車の目標位置 (setpoint) を進める。

        実位置から離れすぎないよう制限する (アンチワインドアップ)。壁に当たって
        止まったときに目標だけが先へ進み、解放された瞬間に大きな力で飛び出す
        のを防ぐ。摩擦 0 の台車なので、この程度の差でも加速力は十分足りる。
        """
        pose = self.odometry_estimator.pose
        setpoint = getattr(self, '_base_setpoint', None)
        if setpoint is None:
            setpoint = (pose.x, pose.y, pose.ang)
        if dt > 0.0:
            setpoint = (
                setpoint[0] + cmd.dot_x * dt,
                setpoint[1] + cmd.dot_y * dt,
                setpoint[2] + cmd.dot_r * dt,
            )
        lag_x = setpoint[0] - pose.x
        lag_y = setpoint[1] - pose.y
        lag_r = math.atan2(
            math.sin(setpoint[2] - pose.ang),
            math.cos(setpoint[2] - pose.ang))
        lag = math.hypot(lag_x, lag_y)
        if _BASE_SETPOINT_MAX_LAG > 0.0 and lag > _BASE_SETPOINT_MAX_LAG:
            scale = _BASE_SETPOINT_MAX_LAG / lag
            lag_x *= scale
            lag_y *= scale
        lag_r = max(-_BASE_SETPOINT_MAX_LAG_ANGULAR,
                    min(_BASE_SETPOINT_MAX_LAG_ANGULAR, lag_r))
        self._base_setpoint = (
            pose.x + lag_x, pose.y + lag_y, pose.ang + lag_r)

    def _kinematic_odom_to_world(self, target):
        """odom 系の平面姿勢 (x, y, yaw) を anchor の world 姿勢へ変換する。

        GT odom の基準変換を逆にたどる。基準取得前だけは現在の base_footprint
        から差分で求めるので、そのフレームだけ dc への問い合わせが要る。
        """
        ref = getattr(self, '_gt_ref', None)
        if ref is not None:
            rwx, rwy, rwyaw, rox, roy, royaw = ref
            odx = target[0] - rox
            ody = target[1] - roy
            coso = math.cos(royaw)
            sino = math.sin(royaw)
            relx = odx * coso + ody * sino
            rely = -odx * sino + ody * coso
            cosw = math.cos(rwyaw)
            sinw = math.sin(rwyaw)
            world_x = rwx + relx * cosw - rely * sinw
            world_y = rwy + relx * sinw + rely * cosw
            world_yaw = rwyaw + math.atan2(
                math.sin(target[2] - royaw),
                math.cos(target[2] - royaw),
            )
            return world_x, world_y, world_yaw

        pose = self.odometry_estimator.pose
        body = getattr(self, '_gt_body', None)
        if not body:
            body = self.dc.get_rigid_body(
                self.stage_path + self.prefix + '/base_footprint')
        current = self.dc.get_rigid_body_pose(body)
        current_yaw = euler_from_quaternion(
            current.r.x, current.r.y, current.r.z, current.r.w)[2]
        frame_yaw = current_yaw - pose.ang
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        world_x = current.p.x + (
            dx * math.cos(frame_yaw) - dy * math.sin(frame_yaw))
        world_y = current.p.y + (
            dx * math.sin(frame_yaw) + dy * math.cos(frame_yaw))
        world_yaw = current_yaw + math.atan2(
            math.sin(target[2] - pose.ang),
            math.cos(target[2] - pose.ang),
        )
        return world_x, world_y, world_yaw

    def _anchor_xform_api(self):
        """anchor prim の XformCommonAPI (生成時と同じ translate + rotateXYZ)。"""
        api = getattr(self, '_base_anchor_xform_api', None)
        if api is not None:
            return api
        prim = stage.get_current_stage().GetPrimAtPath(
            self._base_kinematic_anchor_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(
                'kinematic anchor prim is unavailable: %s'
                % self._base_kinematic_anchor_path)
        api = UsdGeom.XformCommonAPI(prim)
        self._base_anchor_xform_api = api
        return api

    def _verify_anchor_tracking(self, anchor, world_x, world_y):
        """usd モードで anchor が実際に追従しているかを起動直後だけ確かめる。

        USD→PhysX の同期が効かない環境では台車が全く動かなくなるので、指令した
        変位に対して実変位が出ていないことが続いたら dc モードへ戻す。
        """
        if getattr(self, '_base_anchor_mode_verified', False):
            return
        previous_cmd = getattr(self, '_base_anchor_cmd_prev', None)
        previous_actual = getattr(self, '_base_anchor_actual_prev', None)
        self._base_anchor_cmd_prev = (world_x, world_y)
        try:
            pose = self.dc.get_rigid_body_pose(anchor)
            actual = (float(pose.p.x), float(pose.p.y))
        except Exception:
            return
        self._base_anchor_actual_prev = actual
        if previous_cmd is None or previous_actual is None:
            return
        commanded = math.hypot(
            world_x - previous_cmd[0], world_y - previous_cmd[1])
        if commanded < 1e-4:
            return
        moved = math.hypot(
            actual[0] - previous_actual[0], actual[1] - previous_actual[1])
        if moved >= 0.1 * commanded:
            self._base_anchor_mode_verified = True
            return
        fails = getattr(self, '_base_anchor_track_fails', 0) + 1
        self._base_anchor_track_fails = fails
        if fails >= 30:
            self._base_anchor_mode = 'dc'
            self._base_anchor_mode_verified = True
            self._warn_once(
                'kinematic-anchor-mode',
                'USD 経由の kinematic target が反映されないため、'
                'anchor の駆動を dc (瞬間移動) へ戻しました',
            )

    def _set_kinematic_base_pose(self, target):
        """odom の目標姿勢へ kinematic anchor を移す。

        anchor は kinematic body なので速度は書けない (PhysX の
        PxRigidDynamic::setLinearVelocity は "Body must be non-kinematic!" で
        弾き、1000 件でシミュレーション自体が停止する)。代わりに usd モードでは
        USD の xform を書いて PhysX に kinematic target として解釈させ、PhysX 自身
        に速度を導出させる。
        """
        if not all(math.isfinite(float(value)) for value in target):
            self._warn_once(
                'kinematic-nonfinite-target',
                '非有限の台車目標を破棄しました: %r' % (target,),
            )
            return False
        try:
            anchor = getattr(self, '_base_kinematic_anchor', None)
            if not anchor:
                anchor = self.dc.get_rigid_body(
                    self._base_kinematic_anchor_path)
                if not anchor:
                    raise RuntimeError('kinematic anchor handle is unavailable')
                self._base_kinematic_anchor = anchor

            world_x, world_y, world_yaw = self._kinematic_odom_to_world(target)
            # _gt_ref が NaN で汚染されると target が有限でもここが NaN になる。
            # そのまま PhysX へ書くと Invalid PhysX transform でシーンごと発散
            # するので、書く直前にもう一度検査する。
            if not all(math.isfinite(float(value))
                       for value in (world_x, world_y, world_yaw)):
                self._warn_once(
                    'kinematic-nonfinite-world',
                    '非有限の台車 world 姿勢を破棄しました: %r'
                    % ((world_x, world_y, world_yaw),),
                )
                return False

            mode = getattr(self, '_base_anchor_mode', None)
            if mode is None:
                mode = _BASE_KINEMATIC_ANCHOR_MODE
                self._base_anchor_mode = mode
            if mode == 'usd':
                api = self._anchor_xform_api()
                api.SetTranslate(
                    Gf.Vec3d(float(world_x), float(world_y), 0.0))
                api.SetRotate(
                    Gf.Vec3f(0.0, 0.0, math.degrees(float(world_yaw))),
                    UsdGeom.XformCommonAPI.RotationOrderXYZ,
                )
                self._verify_anchor_tracking(anchor, world_x, world_y)
            else:
                transform = _dynamic_control.Transform()
                transform.p = (float(world_x), float(world_y), 0.0)
                half = 0.5 * world_yaw
                transform.r = (0.0, 0.0, math.sin(half), math.cos(half))
                self.dc.set_rigid_body_pose(anchor, transform)
        except Exception as exc:
            self._warn_once(
                'kinematic-drive',
                '決定論的な台車姿勢更新に失敗; 従来制御へ戻します: %s' % exc,
            )
            return False
        return True

    def _physx_scene_query(self):
        """PhysX のシーンクエリインタフェース。使えなければ None。"""
        if hasattr(self, '_physx_query_iface'):
            return self._physx_query_iface
        query = None
        try:
            from omni.physx import get_physx_scene_query_interface
            query = get_physx_scene_query_interface()
        except Exception as exc:
            self._warn_once(
                'kinematic-collision-iface',
                'PhysX シーンクエリを取得できず、台車の衝突クランプを無効化'
                'します: %s' % exc,
            )
        self._physx_query_iface = query
        return query

    def _kinematic_sweep_scale(self, start_world, end_world):
        """start→end の並進を障害物の手前で止める比 (0.0-1.0) を返す。

        anchor は衝突形状を持たず、台車は FixedJoint で無限質量に溶接されている
        ため、PhysX の接触拘束では止まらない。代わりに台車の外周を覆う球を
        スイープし、貫通する分だけ目標の前進量を削る。回転は球スイープでは
        検出できないので通す。
        """
        dx = float(end_world[0]) - float(start_world[0])
        dy = float(end_world[1]) - float(start_world[1])
        distance = math.hypot(dx, dy)
        if distance <= 1e-9 or _BASE_COLLISION_RADIUS <= 0.0:
            return 1.0
        query = self._physx_scene_query()
        if query is None:
            return 1.0

        origin = (float(start_world[0]), float(start_world[1]),
                  _BASE_COLLISION_HEIGHT)
        direction = (dx / distance, dy / distance, 0.0)
        # None のまま帰ってきたら「進路上に何も無い」= クランプしない。
        limit = [None]

        def _report(hit):
            # SceneQueryHitObject の属性は rigid_body / collision (snake_case)。
            # 壁は静的コライダーなので rigid_body は空で、collision だけが埋まる。
            path = str(getattr(hit, 'rigid_body', '')
                       or getattr(hit, 'collision', ''))
            # 自分自身のコライダーは無視する。初期接触 (distance≈0) も無視:
            # 壁際で止まっているときに脱出方向まで塞いでしまうため。余裕
            # (_BASE_COLLISION_MARGIN) を残して停めるので、押し込みは次の
            # ステップのスイープが正の距離で捕まえる。
            if path == '/hsrb' or path.startswith('/hsrb/'):
                return True
            hit_distance = float(getattr(hit, 'distance', 0.0))
            if hit_distance <= 1e-4:
                return True
            if limit[0] is None or hit_distance < limit[0]:
                limit[0] = hit_distance
            return True

        if not self._sweep_sphere(
                query, _BASE_COLLISION_RADIUS, origin, direction, distance,
                _report):
            return 1.0
        if limit[0] is None:
            return 1.0

        allowed = min(distance, max(0.0, limit[0] - _BASE_COLLISION_MARGIN))
        return allowed / distance

    def _sweep_sphere(self, query, radius, origin, direction, distance,
                      report):
        """全ヒットを report へ渡す球スイープ。成功したら True。

        Isaac Sim 4.5 の PhysXSceneQuery は
        sweep_sphere_all(radius, origin, dir, distance, reportFn, bothSides)
        で、ベクトルは carb.Float3。旧名 sweep_sphere も一応試す。
        """
        fn = getattr(self, '_sweep_sphere_fn', None)
        if fn is None:
            for name in ('sweep_sphere_all', 'sweep_sphere'):
                fn = getattr(query, name, None)
                if fn is not None:
                    break
            if fn is None:
                self._warn_once(
                    'kinematic-collision-sweep',
                    '球スイープ API が見つからず、台車の衝突クランプを無効化'
                    'します (利用可能: %s)'
                    % ([n for n in dir(query) if 'sweep' in n],),
                )
                self._physx_query_iface = None
                return False
            self._sweep_sphere_fn = fn

        try:
            import carb
            fn(
                float(radius),
                carb.Float3(*origin),
                carb.Float3(*direction),
                float(distance),
                report,
                False,
            )
        except Exception as exc:
            self._warn_once(
                'kinematic-collision-sweep',
                '台車の衝突スイープに失敗し、クランプを無効化します: %s' % exc,
            )
            # 毎ステップ例外を投げ続けないよう、以後はクランプを止める。
            self._physx_query_iface = None
            self._sweep_sphere_fn = None
            return False
        return True

    def _apply_kinematic_base_motion(self, cmd, dt):
        """平滑化済み速度を積分し、kinematic anchor の平面姿勢を更新する。

        速度書き込みと平面ジョイントの位置ドライブを併用すると拘束ソルバー内で
        競合し、純回転だけでも数十 cm 並進する。移動中は衝突しない anchor の
        目標姿勢だけを進めることで、その競合をなくす。
        """
        if dt <= 0.0:
            return False
        pose = self.odometry_estimator.pose
        motion_values = (
            cmd.dot_x, cmd.dot_y, cmd.dot_r,
            pose.x, pose.y, pose.ang,
        )
        if not all(math.isfinite(float(value)) for value in motion_values):
            # Returning True is intentional: the kinematic path handled the
            # bad command by stopping, so the legacy velocity writer must not
            # receive the same NaN as a fallback.
            self._warn_once(
                'kinematic-nonfinite-command',
                '非有限の台車指令を停止へ置換しました: %r' %
                (motion_values,),
            )
            self._base_direct_pose = None
            self._base_kinematic_velocity = (0.0, 0.0, 0.0)
            return True
        previous = getattr(self, '_base_direct_pose', None)
        if previous is None:
            previous = (pose.x, pose.y, pose.ang)
        candidate = (
            previous[0] + cmd.dot_x * dt,
            previous[1] + cmd.dot_y * dt,
            math.atan2(
                math.sin(previous[2] + cmd.dot_r * dt),
                math.cos(previous[2] + cmd.dot_r * dt),
            ),
        )

        # world 系での前進を求め、障害物を貫通する分だけ削る。world 変換には
        # _gt_ref が要るので、基準取得前 (起動直後の 1 フレーム) は素通しする。
        scale = 1.0
        if _BASE_KINEMATIC_COLLISION and getattr(self, '_gt_ref', None):
            try:
                scale = self._kinematic_sweep_scale(
                    self._kinematic_odom_to_world(previous),
                    self._kinematic_odom_to_world(candidate),
                )
            except Exception as exc:
                self._warn_once(
                    'kinematic-collision',
                    '台車の衝突クランプに失敗し、素通しします: %s' % exc,
                )
                scale = 1.0

        if scale < 1.0:
            target = (
                previous[0] + (candidate[0] - previous[0]) * scale,
                previous[1] + (candidate[1] - previous[1]) * scale,
                candidate[2],
            )
        else:
            target = candidate

        if not self._set_kinematic_base_pose(target):
            self._base_direct_pose = None
            self._base_kinematic_velocity = (0.0, 0.0, 0.0)
            return False

        # クランプ後の姿勢を目標として持ち越す。これが kinematic 経路の
        # アンチワインドアップで、壁の向こうへ目標が積分され続けるのと、
        # 解放された瞬間に飛び出すのを防ぐ。
        self._base_direct_pose = target
        self._base_setpoint = target
        # odom の twist はここから出る (下流で /odom へ流れる)。指令値ではなく
        # 実現できた速度を返し、壁で止まったことをナビゲーション側から
        # 観測できるようにする。
        self._base_kinematic_velocity = (
            cmd.dot_x * scale, cmd.dot_y * scale, cmd.dot_r)
        # クランプの出入りだけを出す (60Hz で毎ステップ出すとログが埋まる)。
        clamped = scale < 1.0
        if _BASE_DIAG and clamped != getattr(self, '_base_clamp_prev', False):
            print(
                '[base-diag] kinematic collision clamp %s scale=%.3f '
                'cmd=(%.3f, %.3f, %.3f)'
                % ('ON' if clamped else 'OFF', scale,
                   cmd.dot_x, cmd.dot_y, cmd.dot_r),
                flush=True,
            )
        self._base_clamp_prev = clamped
        # 平面ジョイント側も同じ姿勢へ合わせ、次の物理ステップで姿勢更新を
        # 打ち消す力が出ないようにする。
        self._set_joint_brake_target(target)
        return True

    def _odom_to_joint(self, pose):
        """odom 姿勢 → 平面ジョイントの DOF 値。

        ジョイントの原点は spawn 姿勢 (frame0)、odom の原点は起動直後に
        真値odomを初めて読んだ時点の姿勢で、両者は数 cm ずれる。この差を
        足さないとサーボの指令が常にずれ、台車が目標に到達できなくなる
        (実測: 6.6cm ずれた状態で reset 後の動作が全て失敗した)。
        """
        ref = getattr(self, '_gt_ref', None)
        frame0 = getattr(self, '_planar_frame0', None)
        if ref is None or frame0 is None:
            return pose
        # ref = (world_x, world_y, world_yaw, odom_x, odom_y, odom_yaw)
        # odom 原点に対応する world 姿勢と frame0 の差が、そのままオフセット。
        return (
            pose[0] + (ref[0] - ref[3]) - frame0[0],
            pose[1] + (ref[1] - ref[4]) - frame0[1],
            pose[2] + (ref[2] - ref[5]) - frame0[2],
        )

    def _set_joint_brake_target(self, pose, velocity=(0.0, 0.0, 0.0)):
        """平面ジョイントのドライブ目標 (位置・速度) を設定する。

        引数は odom 姿勢。内部でジョイントの DOF 値へ直してから渡す。
        出力 = 剛性 x (目標位置 - 現在位置) + 減衰 x (目標速度 - 現在速度)。
        停止中は目標速度 0 で現在位置に固定 = 駐車ブレーキ。走行中は指令速度を
        目標速度に入れておけば減衰項が走行を妨げない。
        """
        drives = getattr(self, '_base_brake_drives', None)
        if not drives:
            return
        pose = self._odom_to_joint(pose)
        state = (tuple(pose), tuple(velocity))
        if getattr(self, '_base_brake_drive_state', None) == state:
            return
        try:
            drives['transX'].GetTargetPositionAttr().Set(float(pose[0]))
            drives['transY'].GetTargetPositionAttr().Set(float(pose[1]))
            drives['rotZ'].GetTargetPositionAttr().Set(
                math.degrees(math.atan2(
                    math.sin(pose[2]), math.cos(pose[2]))))
            drives['transX'].GetTargetVelocityAttr().Set(float(velocity[0]))
            drives['transY'].GetTargetVelocityAttr().Set(float(velocity[1]))
            drives['rotZ'].GetTargetVelocityAttr().Set(
                math.degrees(velocity[2]))
            self._base_brake_drive_state = state
        except Exception as exc:
            self._warn_once(
                'brake-drive', 'ブレーキドライブ目標の設定に失敗: %s' % exc)

    def _warn_once(self, key, message):
        seen = getattr(self, '_warned_keys', None)
        if seen is None:
            seen = set()
            self._warned_keys = seen
        if key in seen:
            return
        seen.add(key)
        print('[base-brake] ' + message, flush=True)

    def _release_base_brake(self, reason=''):
        if not getattr(self, '_base_brake_engaged', False):
            self._base_brake_pin = None
            return
        self._base_brake_engaged = False
        self._base_brake_pin = None
        # 走行再開時に古い目標へ引かれないよう、設定値を実位置に戻す。
        self._base_setpoint = (
            self.odometry_estimator.pose.x,
            self.odometry_estimator.pose.y,
            self.odometry_estimator.pose.ang,
        )
        if _BASE_DIAG:
            print('[base-brake] 保持解除%s'
                  % ((' (%s)' % reason) if reason else ''), flush=True)

    def _base_integral_command(self, dx, dy, dyaw, dt):
        if _BASE_TRAJ_I_GAIN <= 0.0 or dt <= 0.0:
            return (0.0, 0.0, 0.0)

        integral = self._base_pose_error_integral
        errors = (dx, dy, dyaw)
        for index, error in enumerate(errors):
            # Callers pass zero inside the control deadband.  Keeping the old
            # integral there drove the base through an already-reached goal.
            # Also discard compensation after crossing the target so it cannot
            # sustain a slow endpoint oscillation.
            if error == 0.0 or integral[index] * error < 0.0:
                integral[index] = 0.0
            else:
                integral[index] += error * dt

        linear_limit = (
            _BASE_TRAJ_I_LINEAR_LIMIT / _BASE_TRAJ_I_GAIN)
        linear_norm = math.hypot(integral[0], integral[1])
        if linear_limit <= 0.0:
            integral[0] = 0.0
            integral[1] = 0.0
        elif linear_norm > linear_limit:
            scale = linear_limit / linear_norm
            integral[0] *= scale
            integral[1] *= scale

        angular_limit = (
            _BASE_TRAJ_I_ANGULAR_LIMIT / _BASE_TRAJ_I_GAIN)
        integral[2] = max(
            -angular_limit, min(angular_limit, integral[2]))
        return tuple(_BASE_TRAJ_I_GAIN * value for value in integral)

    def reset_to_spawn(self, x, y, yaw):
        """ロボットを spawn 姿勢 (x, y, yaw) + ホーム関節へ戻し,速度をゼロに"""
        if not getattr(self, 'art', None):
            return

        # --- base (articulation root) をテレポートして速度ゼロ ---
        # Dynamic Control の剛体テレポートはルートリンクだけを動かすため、
        # 隣接リンクとの拘束が大きく破れ、PhysX がそれを解消する際にロボットが
        # 数 m 弾き飛ばされる (実測)。関節全体を一括で移動できる Tensor API を
        # 優先し、失敗したときだけ従来の方法にフォールバックする。
        _half = 0.5 * float(yaw)
        moved = False
        try:
            self.robots.set_world_poses(
                np.array([[float(x), float(y), 0.0]], dtype=np.float32),
                # Isaac の四元数は (w, x, y, z) 順
                np.array([[math.cos(_half), 0.0, 0.0, math.sin(_half)]],
                         dtype=np.float32),
            )
            self.robots.set_velocities(np.zeros((1, 6), dtype=np.float32))
            moved = True
        except Exception as exc:
            self._warn_once('reset-pose', 'リセットの一括移動に失敗: %s' % exc)

        if _BASE_KINEMATIC_DRIVE:
            try:
                anchor = getattr(self, '_base_kinematic_anchor', None)
                if not anchor:
                    anchor = self.dc.get_rigid_body(
                        self._base_kinematic_anchor_path)
                    self._base_kinematic_anchor = anchor
                anchor_pose = _dynamic_control.Transform()
                anchor_pose.p = (float(x), float(y), 0.0)
                anchor_pose.r = (
                    0.0, 0.0, math.sin(_half), math.cos(_half))
                self.dc.set_rigid_body_pose(anchor, anchor_pose)
            except Exception as exc:
                self._warn_once(
                    'reset-anchor',
                    'kinematic anchor のリセットに失敗: %s' % exc,
                )

        root = self.dc.get_articulation_root_body(self.art)
        if not moved:
            t = _dynamic_control.Transform()
            t.p = (float(x), float(y), 0.0)
            # dc の Transform.r は (x, y, z, w)。yaw (Z 軸) のみの回転。
            t.r = (0.0, 0.0, math.sin(_half), math.cos(_half))
            self.dc.set_rigid_body_pose(root, t)
        self.dc.set_rigid_body_linear_velocity(root, (0.0, 0.0, 0.0))
        self.dc.set_rigid_body_angular_velocity(root, (0.0, 0.0, 0.0))

        # --- 全関節をホーム姿勢へ・速度ゼロ・目標も合わせる ---
        home = getattr(self, '_home_dof_pos', {})
        for _name, (_ptr, _jt, _inv) in self._joints.items():
            _pos = home.get(_name, 0.0)
            self.dc.set_dof_position(_ptr, _pos)
            self.dc.set_dof_velocity(_ptr, 0.0)
            self.dc.set_dof_position_target(_ptr, _pos)
            self.dc.set_dof_velocity_target(_ptr, 0.0)

        # Base drives keep fixed velocity-drive properties for the articulation
        # lifetime.  Changing DOF properties while PhysX is running invalidates
        # the dynamic-control articulation handle in Isaac Sim 4.5.
        self._base_cmd_prev = JointSpace()
        self._base_direct_cmd_prev = CartSpace()
        self._base_direct_applied_cmd = CartSpace()
        self._base_direct_pose = None
        self._base_kinematic_velocity = (0.0, 0.0, 0.0)
        self._base_hold_pose = None
        # テレポート後に旧位置へバネで引き戻されないよう必ず解除し、
        # 平面ジョイントのドライブ目標も spawn 姿勢 (= joint frame0 の原点) へ
        # 移す。これをしないと剛性 20万 N/m のバネが旧位置へ引き戻し続け、
        # reset_world が効かなくなる。
        # 実行中の軌道は破棄する。残しておくとリセット後にロボットが
        # 「リセット前の目標」へ戻ろうとして、せっかく戻した位置から動き出す。
        for _srv in (
            getattr(self, 'odom_trajectory_action_server', None),
            getattr(self, 'arm_trajectory_action_server', None),
            getattr(self, 'head_trajectory_action_server', None),
            getattr(self, 'gripper_trajectory_action_server', None),
        ):
            if _srv is None:
                continue
            _handle = getattr(_srv, '_action_goal_handle', None)
            _srv._action_goal = None
            _srv._pending_topic_msg = None
            if _handle is not None and _handle.is_active:
                try:
                    _handle.abort()
                except Exception:
                    pass
            _srv._action_goal_handle = None
        self.cmd_vel_msg = None
        self.last_cmd_vel_time = 0.0

        # テレポートで Dynamic Control のハンドルが無効になると、真値odomの
        # 取得が例外で落ち、古い値のまま積分し続けて座標が壊れる。壊れた座標を
        # 位置サーボの目標に使うと台車が明後日の方向へ引かれるので、ハンドルを
        # 取り直し、オドメトリも spawn 原点へ戻す。
        self._gt_body = None
        self.odometry_estimator.pose.x = 0.0
        self.odometry_estimator.pose.y = 0.0
        self.odometry_estimator.pose.ang = 0.0

        self._release_base_brake()
        self._base_setpoint = (0.0, 0.0, 0.0)
        self._set_joint_brake_target((0.0, 0.0, 0.0))
        # 全関節を一瞬でホーム姿勢へ戻すと、その位置の飛びを PhysX が解消する
        # ときに大きな力が出て台車が弾かれる。落ち着くまでの短い間だけ、誤差の
        # 大小によらず spawn 位置へ固定し続ける。
        self._base_hold_pose = (0.0, 0.0, 0.0)
        self._post_reset_hold_until = (
            self.simulation_context.current_time + _BASE_RESET_HOLD_TIME)
        self._base_pose_error_integral = [0.0, 0.0, 0.0]
        self._base_trajectory_was_active = False
        self._joint_state_previous_positions = {}
        self._joint_state_filtered_velocities = {}

        self.dc.wake_up_articulation(self.art)

        # --- オドメトリ・速度指令をリセット ---
        # odom is local to the spawn pose.  Using world-space spawn x/y here
        # adds the placement offset a second time when _gt_ref is recaptured.
        self.odometry_estimator.set_pose(0.0, 0.0, 0.0)
        self.cmd_vel_msg = None
        self._gt_ref = None   # 真値odomの基準を再取得(テレポート後に対応づけ直す)

        # --- 把持中の物体があれば離す (衝突を戻し、追従を解除) ---
        if getattr(self, '_grasp_obj', None) is not None:
            try:
                self._set_grasp_object_collision(self._grasp_obj['path'], True)
            except Exception:
                pass
            self._grasp_obj = None

    def step(self):
        if is_ros2:
            # Publish simulation time on every physics/control step.  Camera and
            # viewport rendering can then be decimated without reducing the
            # clock rate seen by MoveIt trajectory interpolation.
            clock = Clock()
            clock.clock = self.get_ros_time(
                self.simulation_context.current_time)
            self.clock_pub.publish(clock)
        else:
            og.Controller.set(
                og.Controller.attribute(
                    '/ros_controllers/OnImpulseEvent.state:enableImpulse'), True
            )

        dt = self.simulation_context.current_time - self.prev_time

        if not self.art:
            self.art = self.dc.get_articulation(self.stage_path + self.prefix)
            if self.art == _dynamic_control.INVALID_HANDLE:
                print('{self.prefix} is not an articulation')
            self._joints = {}
            for i in range(self.dc.get_articulation_dof_count(self.art)):
                dof_ptr = self.dc.get_articulation_dof(self.art, i)
                if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                    dof_name = self.dc.get_dof_name(dof_ptr)
                    joint = self.dc.find_articulation_joint(self.art, dof_name)
                    self._joints[dof_name] = (
                        dof_ptr,
                        self.dc.get_joint_type(joint),
                        dof_name
                        in [
                            'arm_flex_joint',
                            'arm_lift_joint',
                            'wrist_flex_joint',
                            'arm_roll_joint',
                        ],
                    )
            print(self._joints)
            self.left_wheel_ptr = self.dc.find_articulation_dof(
                self.art, 'base_l_drive_wheel_joint'
            )
            self.right_wheel_ptr = self.dc.find_articulation_dof(
                self.art, 'base_r_drive_wheel_joint'
            )
            self.roll_ptr = self.dc.find_articulation_dof(
                self.art, 'base_roll_joint')
            self._base_root_body = self.dc.get_articulation_root_body(self.art)
            self.robots.initialize()

            for _name in (
                'arm_lift_joint',
                'torso_lift_joint',
                'arm_flex_joint',
                'arm_roll_joint',
                'wrist_flex_joint',
                'wrist_roll_joint',
            ):
                _ptr = self._joints[_name][0]
                _props = self.dc.get_dof_properties(_ptr)
                print(
                    '[arm-runtime] %s k=%.6g d=%.6g maxEffort=%.6g '
                    'maxVelocity=%.6g'
                    % (
                        _name,
                        _props.stiffness,
                        _props.damping,
                        _props.max_effort,
                        _props.max_velocity,
                    ),
                    flush=True,
                )

            # for reset world
            self._home_dof_pos = {
                _n: self.dc.get_dof_state(
                    _p[0], _dynamic_control.STATE_POS).pos
                for _n, _p in self._joints.items()
            }

            if _BASE_PHYSICS_REPORT:
                self._dump_physics_report()

        self.dc.wake_up_articulation(self.art)

        # 物体は spawn 後に落下・転がりで位置が変わる。落ち着いた頃に一度だけ
        # 実際の座標を出しておくと、把持の目標値をそこから決められる。
        if (
            _OBJECT_POSE_REPORT
            and not getattr(self, '_obj_pose_reported', False)
            and self.simulation_context.current_time > _OBJECT_POSE_REPORT_AT
        ):
            self._obj_pose_reported = True
            self._dump_object_poses()

        self.publish_joint_states(dt)

        # 案A: グリッパが閉じて物体が近ければ掴んで追従、開いたら離す
        # (正攻法=物理衝突の検証中は _ATTACH_GRASP_ENABLED=False で無効化)
        if _ATTACH_GRASP_ENABLED:
            self._grasp_attach_update()

        left_state = self.dc.get_dof_state(
            self.left_wheel_ptr, _dynamic_control.STATE_ALL)
        right_state = self.dc.get_dof_state(
            self.right_wheel_ptr, _dynamic_control.STATE_ALL)
        roll_state = self.dc.get_dof_state(
            self.roll_ptr, _dynamic_control.STATE_ALL)

        state_ = VehicleState()
        state_.steer_angle = roll_state.pos
        joint_param_ = JointSpace()
        joint_param_.vel_wheel_l = left_state.vel
        joint_param_.vel_wheel_r = right_state.vel
        joint_param_.vel_steer = roll_state.vel

        if _BASE_DIRECT_DRIVE:
            # In direct mode the wheel joints are visual/passive; odometry
            # velocity must come from the articulation root, not wheel motion.
            # Read the same Tensor API state used below for commands; the
            # Dynamic Control rigid-body setter is reduced by the fixed-link
            # constraint solve before it becomes articulation root velocity.
            cartesian_param_ = CartSpace()
            if _BASE_KINEMATIC_DRIVE:
                _kv = getattr(
                    self, '_base_kinematic_velocity', (0.0, 0.0, 0.0))
                abs_dot_x, abs_dot_y = float(_kv[0]), float(_kv[1])
                cartesian_param_.dot_r = float(_kv[2])
            else:
                root_velocity = self.robots.get_velocities()[0]
                abs_dot_x = float(root_velocity[0])
                abs_dot_y = float(root_velocity[1])
                cartesian_param_.dot_r = float(root_velocity[5])
            if not _GT_ODOM and dt > 0.0:
                self.odometry_estimator.pose.x += abs_dot_x * dt
                self.odometry_estimator.pose.y += abs_dot_y * dt
                self.odometry_estimator.pose.ang += (
                    cartesian_param_.dot_r * dt)
        else:
            cartesian_param_ = self.vehicle_dynamics.forward(
                joint_param_, state_)
            abs_dot_x, abs_dot_y = self.odometry_estimator.integrate(
                cartesian_param_, dt)

        # === Phase2: odometry_estimator.pose を物理の真値で上書き ===
        # これ以降の wheel_odom発行(2195)・/omni_base_controller/state・odom_x/y/t(→計画)・
        # 軌道追従制御 が全て真値の台車位置を使う(車輪スリップ/レーザ破綻を回避)。
        # twist(abs_dot_x/y)は integrate 値のまま。回転中心 base_footprint を読む。
        # odom≠world のオフセットは初回に一度だけ 真world↔現在odom を記録して吸収。
        if _GT_ODOM:
            try:
                if getattr(self, '_gt_body', None) is None:
                    _bf = self.dc.get_rigid_body(
                        self.stage_path + self.prefix + '/base_footprint')
                    self._gt_body = _bf if _bf else \
                        self.dc.get_articulation_root_body(self.art)
                _gp = self.dc.get_rigid_body_pose(self._gt_body)
                _wx, _wy = float(_gp.p.x), float(_gp.p.y)
                _wyaw = euler_from_quaternion(
                    _gp.r.x, _gp.r.y, _gp.r.z, _gp.r.w)[2]
                # PhysX が一度でも発散すると真値姿勢が NaN になる。NaN は例外を
                # 出さないので下の except では捕まらず、そのまま _gt_ref に
                # 焼き付くと以降の odom も kinematic 目標も恒久的に NaN 化して
                # sim の再起動が要る状態になる。基準に採る前に必ず弾く。
                if not all(math.isfinite(_v) for _v in (_wx, _wy, _wyaw)):
                    raise ValueError(
                        '非有限の真値 odom: (%r, %r, %r)' % (_wx, _wy, _wyaw))
                if getattr(self, '_gt_ref', None) is None:
                    self._gt_ref = (_wx, _wy, _wyaw,
                                    self.odometry_estimator.pose.x,
                                    self.odometry_estimator.pose.y,
                                    self.odometry_estimator.pose.ang)
                _rwx, _rwy, _rwyaw, _rox, _roy, _royaw = self._gt_ref
                _ddx, _ddy = _wx - _rwx, _wy - _rwy
                _rc, _rs = math.cos(_rwyaw), math.sin(_rwyaw)
                _relx = _ddx * _rc + _ddy * _rs
                _rely = -_ddx * _rs + _ddy * _rc
                _relyaw = math.atan2(
                    math.sin(_wyaw - _rwyaw), math.cos(_wyaw - _rwyaw))
                _oc, _os = math.cos(_royaw), math.sin(_royaw)
                self.odometry_estimator.pose.x = _rox + _relx * _oc - _rely * _os
                self.odometry_estimator.pose.y = _roy + _relx * _os + _rely * _oc
                self.odometry_estimator.pose.ang = math.atan2(
                    math.sin(_royaw + _relyaw), math.cos(_royaw + _relyaw))
            except Exception as exc:
                # 真値が取れないフレームは前回の odom を据え置く。黙って捨てると
                # 発散に気付けないので一度だけ通知する。
                self._warn_once(
                    'gt-odom', '真値 odom の取り込みに失敗: %s' % exc)

        odom = Odometry()
        odom.header.stamp = self.get_ros_time(
            self.simulation_context.current_time)
        odom.header.frame_id = 'odom' if is_ros2 else 'world'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.odometry_estimator.pose.x
        odom.pose.pose.position.y = self.odometry_estimator.pose.y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.odometry_estimator.pose.ang)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.pose.covariance = [
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1000.0,
        ]
        odom.twist.twist.linear.x = abs_dot_x
        odom.twist.twist.linear.y = abs_dot_y
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = cartesian_param_.dot_r
        odom.twist.covariance = [
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            100000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1000.0,
        ]
        self.base_odom_pub.publish(odom)

        cmd = CartSpace()
        base_trajectory_active_now = (
            self.odom_trajectory_action_server._action_goal is not None)
        base_trajectory_was_active = self._base_trajectory_was_active
        if base_trajectory_was_active and not base_trajectory_active_now:
            # cmd_vel received while MoveIt owned the base was ignored during
            # execution. Do not replay its two-second cache after the action;
            # only a fresh teleoperation message may release endpoint hold.
            self.cmd_vel_msg = None
            self.last_cmd_vel_time = 0.0
            # The action only succeeds below the endpoint velocity tolerance.
            # Clear the trajectory filter at that point so its residual command
            # cannot carry the base past the pose captured for endpoint hold.
            self._base_direct_cmd_prev = CartSpace()
            self._base_direct_applied_cmd = CartSpace()
            # Once a trajectory has succeeded inside the configured tolerance,
            # hold the accepted physical pose.  Continuing to pull toward the
            # mathematical endpoint caused millimetre-scale post-action creep.
            if self._base_hold_pose is not None:
                hold_x, hold_y, hold_yaw = self._base_hold_pose
                actual_pose = self.odometry_estimator.pose
                hold_errors = (
                    hold_x - actual_pose.x,
                    hold_y - actual_pose.y,
                    math.atan2(
                        math.sin(hold_yaw - actual_pose.ang),
                        math.cos(hold_yaw - actual_pose.ang),
                    ),
                )
                if max(abs(error) for error in hold_errors) <= (
                    _BASE_GOAL_TOLERANCE + 1e-6
                ):
                    self._base_hold_pose = (
                        actual_pose.x,
                        actual_pose.y,
                        actual_pose.ang,
                    )
                    self._reset_base_pose_integral()
        manual_cmd_active = (
            self.last_cmd_vel_time + _BASE_CMD_VEL_TIMEOUT
            > self.simulation_context.current_time
            and self.cmd_vel_msg is not None
            and (
                abs(self.cmd_vel_msg.linear.x) > 1e-6
                or abs(self.cmd_vel_msg.linear.y) > 1e-6
                or abs(self.cmd_vel_msg.angular.z) > 1e-6
            )
        )
        # 駐車ブレーキの目標 (odom 姿勢)。None ならブレーキ解除。
        # 「台車の軌道アクションが動いているか」ではなく「台車が今止まっている
        # べきか」で判断する。whole_body は腕だけ動かすときも台車コントローラへ
        # 「その場に留まれ」という軌道を送るため、前者で判定すると腕を振る間ずっと
        # ブレーキが解除され、反力で流されてしまう。
        brake_target = None
        brake_reason = ''
        # リセット直後は無条件で spawn 位置へ固定する (関節を戻す衝撃対策)。
        post_reset_hold = (
            self.simulation_context.current_time
            < getattr(self, '_post_reset_hold_until', 0.0))
        if base_trajectory_active_now:
            srv = self.odom_trajectory_action_server
            srv._odometry = self.odometry_estimator.pose
            # Keep the final trajectory target after all points are consumed.
            # The action server waits for odometry convergence; replacing this
            # target with the current pose made every under-tracked motion look
            # successful and stopped the base short of the MoveIt goal.
            # === 本物の OmniBaseController と同じ制御則 ===
            #   出力速度 = 目標速度(FF) + p_gain × 姿勢誤差(FB)   (odom 系)
            # FF: 軌道点が運ぶ「速度」をサンプリング(差分自作はスパイク暴発するので不可)。
            # FB: 目標位置 - 現在位置。向き誤差は必ず -pi..pi に wrap(π跨ぎ/角累積で
            #     ~2pi になり遠回りスピンするのを防ぐ)。
            _ffx, _ffy, _ffr = srv.sample_desired_velocity()
            _vx, _vy, _vr = (
                getattr(srv, '_velocity', None) or (0.0, 0.0, 0.0)
            )
            # Direct drive では whole-body の反力に追従負けしないよう、計測済みの
            # p_gain=2.0 を既定値にする。平面外の揺れは planar joint が拘束する。
            _dx = srv._joints['odom_x'] - self.odometry_estimator.pose.x
            _dy = srv._joints['odom_y'] - self.odometry_estimator.pose.y
            cmd.dot_x = (
                _ffx
                + _BASE_TRAJ_P_GAIN * _dx
                + _BASE_TRAJ_D_GAIN * (_ffx - _vx)
            )
            cmd.dot_y = (
                _ffy
                + _BASE_TRAJ_P_GAIN * _dy
                + _BASE_TRAJ_D_GAIN * (_ffy - _vy)
            )
            _dr = math.atan2(
                math.sin(
                    srv._joints['odom_t'] - self.odometry_estimator.pose.ang),
                math.cos(
                    srv._joints['odom_t'] - self.odometry_estimator.pose.ang),
            )
            cmd.dot_r = (
                _ffr
                + _BASE_TRAJ_P_GAIN * math.atan2(
                    math.sin(_dr), math.cos(_dr))
                + _BASE_TRAJ_D_GAIN * (_ffr - _vr)
            )
            # Stop only after the pose itself is inside the action tolerance.
            # A velocity deadband left a steady error of deadband / P-gain
            # (about 22 mm with the previous 0.01 / 0.45 settings).
            if abs(_ffx) < 1e-4 and abs(_dx) <= _BASE_CONTROL_DEADBAND:
                cmd.dot_x = -_BASE_TRAJ_D_GAIN * _vx
            if abs(_ffy) < 1e-4 and abs(_dy) <= _BASE_CONTROL_DEADBAND:
                cmd.dot_y = -_BASE_TRAJ_D_GAIN * _vy
            if abs(_ffr) < 1e-4 and abs(_dr) <= _BASE_CONTROL_DEADBAND:
                cmd.dot_r = -_BASE_TRAJ_D_GAIN * _vr

            # While following the time-parameterized path, FF+P avoids integral
            # windup.  At the final point, add a bounded integral term so the
            # static reaction from an extended arm cannot leave the base a
            # couple of centimetres short of the MoveIt endpoint.
            trajectory_done = (
                srv._action_point_index
                >= len(srv._action_goal.trajectory.points)
            )
            if trajectory_done:
                _ix, _iy, _ir = self._base_integral_command(
                    0.0 if abs(_dx) <= _BASE_CONTROL_DEADBAND else _dx,
                    0.0 if abs(_dy) <= _BASE_CONTROL_DEADBAND else _dy,
                    0.0 if abs(_dr) <= _BASE_CONTROL_DEADBAND else _dr,
                    dt,
                )
                cmd.dot_x += _ix
                cmd.dot_y += _iy
                cmd.dot_r += _ir
            else:
                self._reset_base_pose_integral()

            # 軌道自体が台車を動かそうとしていない (目標速度ほぼ 0) 区間で、
            # かつ目標付近にいるならブレーキで固定する。腕だけを動かす
            # whole_body 動作はこの条件に入り続けるので終始台車が動かない。
            if (
                trajectory_done
                and max(abs(_ffx), abs(_ffy), abs(_ffr)) < 1e-3
                and math.hypot(_dx, _dy) <= _BASE_BRAKE_ENGAGE_LINEAR
                and abs(_dr) <= _BASE_BRAKE_ENGAGE_ANGULAR
            ):
                brake_target = (
                    srv._joints['odom_x'],
                    srv._joints['odom_y'],
                    srv._joints['odom_t'],
                )
            else:
                brake_reason = (
                    '軌道追従中 ff=%.3f 誤差=%.3fm/%.3frad'
                    % (max(abs(_ffx), abs(_ffy), abs(_ffr)),
                       math.hypot(_dx, _dy), abs(_dr)))
            # Keep the last desired pose after the action completes.  Direct
            # drive has no wheel-ground traction, so releasing the chassis here
            # lets arm reaction forces move it away from the reached goal.
            self._base_hold_pose = (
                srv._joints['odom_x'],
                srv._joints['odom_y'],
                srv._joints['odom_t'],
            )
        elif manual_cmd_active:
            self._base_hold_pose = None  # 手動指令中はホールド解除
            brake_reason = 'cmd_vel 受信中'
            self._reset_base_pose_integral()
            ang = self.odometry_estimator.pose.ang + 0.5 * self.cmd_vel_msg.angular.z * dt
            cosr = math.cos(ang)
            sinr = math.sin(ang)
            cmd.dot_x = self.cmd_vel_msg.linear.x * cosr - self.cmd_vel_msg.linear.y * sinr
            cmd.dot_y = self.cmd_vel_msg.linear.x * sinr + self.cmd_vel_msg.linear.y * cosr
            cmd.dot_r = self.cmd_vel_msg.angular.z
        else:
            cmd.dot_x = 0.0
            cmd.dot_y = 0.0
            cmd.dot_r = 0.0
            if _BASE_DIRECT_DRIVE:
                # Keep holding the planned endpoint.  Capturing the current
                # pose here makes a failed or slightly under-tracked action
                # permanently stop short.
                if self._base_hold_pose is None:
                    self._base_hold_pose = (
                        self.odometry_estimator.pose.x,
                        self.odometry_estimator.pose.y,
                        self.odometry_estimator.pose.ang,
                    )
                    self._reset_base_pose_integral()
                if base_trajectory_was_active:
                    print(
                        '[base-hold] target=(%.4f, %.4f, %.4f)'
                        % self._base_hold_pose,
                        flush=True,
                    )
                hx, hy, ht = self._base_hold_pose
                hold_dx = hx - self.odometry_estimator.pose.x
                hold_dy = hy - self.odometry_estimator.pose.y
                hold_dr = math.atan2(
                    math.sin(ht - self.odometry_estimator.pose.ang),
                    math.cos(ht - self.odometry_estimator.pose.ang),
                )
                # 常にブレーキで保持する。遠い場合も目標を毎回いまの位置へ
                # 付け替えると、外力で押されるたびに保持位置がずれていき、
                # 台車が滑り続けてしまう (reset 後に実測)。保持位置は固定し、
                # そこへ creep 速度でゆっくり戻す。
                brake_target = self._base_hold_pose
        # 軌道追従していない間(cmd_vel/静止)は /omni_base_controller/state の actual(=_joints)を
        # 現在の真値odomで更新し続ける。これをしないと計画が読む台車位置が古い(0)ままで、
        # 台車が cmd_vel(nav)で動いた後の whole_body 計画が間違った位置から立ち、台車が暴れる。
        if self.odom_trajectory_action_server._action_goal is None:
            _sj = self.odom_trajectory_action_server._joints
            _sj['odom_x'] = self.odometry_estimator.pose.x
            _sj['odom_y'] = self.odometry_estimator.pose.y
            _sj['odom_t'] = self.odometry_estimator.pose.ang

        relcmd = CartSpace()
        diff_r = cmd.dot_r * dt
        ang = self.odometry_estimator.pose.ang + 0.5 * diff_r
        cosr = math.cos(-ang)
        sinr = math.sin(-ang)
        relcmd.dot_x = cmd.dot_x * cosr - cmd.dot_y * sinr
        relcmd.dot_y = cmd.dot_x * sinr + cmd.dot_y * cosr
        # diff_r = cmd.dot_r * dt なので diff_r/dt は数学的に cmd.dot_r と同じ。
        # ただし dt=0(タイムライン一時停止中など)だと 0/0 でゼロ割りクラッシュになり、
        # step() が毎フレーム落ちてログが溢れる。割り算をやめて等価な cmd.dot_r を使う。
        relcmd.dot_r = cmd.dot_r

        jcmd = self.vehicle_dynamics.inverse(relcmd, state_)

        ratio = abs(jcmd.vel_steer) / self.vel_limit_steer_
        ratio = max(ratio, abs(jcmd.vel_wheel_l) / self.vel_limit_wheel_)
        ratio = max(ratio, abs(jcmd.vel_wheel_r) / self.vel_limit_wheel_)
        if ratio > 1.0:
            jcmd.vel_steer /= ratio
            jcmd.vel_wheel_l /= ratio
            jcmd.vel_wheel_r /= ratio

        _base_trajectory_active = base_trajectory_active_now
        self._base_trajectory_was_active = _base_trajectory_active
        _base_idle = (
            not _base_trajectory_active and
            abs(cmd.dot_x) < 1e-6 and
            abs(cmd.dot_y) < 1e-6 and
            abs(cmd.dot_r) < 1e-6
        )

        if post_reset_hold:
            # リセット直後にブレーキを掛けると、テレポート直後の拘束状態と
            # バネ目標が噛み合わず台車が飛ばされる (実測で毎回同じ位置へ 2m)。
            # 落ち着くまではブレーキを解除し、ドライブ目標を実位置に追従させて
            # 力が出ない状態にしておく。
            # 全関節を一瞬でホーム姿勢へ戻す衝撃で台車が動くので、その間は
            # 誤差の大小によらず spawn 位置へ固定し続ける。
            brake_target = (0.0, 0.0, 0.0)
            self._base_hold_pose = (0.0, 0.0, 0.0)
            cmd = CartSpace()
        if _BASE_DIRECT_DRIVE and _BASE_BRAKE_ENABLED and brake_target is not None:
            # --- 駐車ブレーキ: 姿勢を固定して腕の反力に流されないようにする ---
            self._base_direct_cmd_prev = CartSpace()
            self._base_direct_applied_cmd = CartSpace()
            self._base_direct_pose = None
            self._base_kinematic_velocity = (0.0, 0.0, 0.0)
            self._apply_base_brake(brake_target, dt)
            if _BASE_KINEMATIC_DRIVE:
                # 停止時だけ物理ブレーキへ戻すと、移動中に消したソルバー競合が
                # 再発し、停止後に数十 cm 揺り戻される。ブレーキの creep で求めた
                # pin を articulation 姿勢にも反映して、停止保持も決定論的にする。
                self._set_kinematic_base_pose(self._base_brake_pin)
            self.dc.set_dof_velocity_target(self.left_wheel_ptr, 0.0)
            self.dc.set_dof_velocity_target(self.right_wheel_ptr, 0.0)
            self.dc.set_dof_velocity_target(self.roll_ptr, 0.0)
        elif _BASE_DIRECT_DRIVE:
            self._release_base_brake(brake_reason)
            direct_cmd = getattr(self, '_base_direct_cmd_prev', CartSpace())
            if dt > 0.0:
                alpha = min(1.0, dt / (_BASE_CMD_TAU + dt))
                filtered = CartSpace()
                filtered.dot_x = direct_cmd.dot_x + alpha * (
                    cmd.dot_x - direct_cmd.dot_x)
                filtered.dot_y = direct_cmd.dot_y + alpha * (
                    cmd.dot_y - direct_cmd.dot_y)
                filtered.dot_r = direct_cmd.dot_r + alpha * (
                    cmd.dot_r - direct_cmd.dot_r)

                delta_x = filtered.dot_x - direct_cmd.dot_x
                delta_y = filtered.dot_y - direct_cmd.dot_y
                delta_r = filtered.dot_r - direct_cmd.dot_r
                accel_scale = 1.0
                delta_linear = math.hypot(delta_x, delta_y)
                if _BASE_LINEAR_ACCEL_LIMIT > 0.0 and delta_linear > 0.0:
                    accel_scale = min(
                        accel_scale,
                        _BASE_LINEAR_ACCEL_LIMIT * dt / delta_linear,
                    )
                if _BASE_ANGULAR_ACCEL_LIMIT > 0.0 and abs(delta_r) > 0.0:
                    accel_scale = min(
                        accel_scale,
                        _BASE_ANGULAR_ACCEL_LIMIT * dt / abs(delta_r),
                    )

                smoothed = CartSpace()
                smoothed.dot_x = direct_cmd.dot_x + accel_scale * delta_x
                smoothed.dot_y = direct_cmd.dot_y + accel_scale * delta_y
                smoothed.dot_r = direct_cmd.dot_r + accel_scale * delta_r
                if (
                    abs(cmd.dot_x) < 1e-6
                    and abs(cmd.dot_y) < 1e-6
                    and abs(cmd.dot_r) < 1e-6
                    and max(
                        abs(smoothed.dot_x),
                        abs(smoothed.dot_y),
                        abs(smoothed.dot_r),
                    ) < 1e-3
                ):
                    smoothed = CartSpace()
                direct_cmd = smoothed
                self._base_direct_cmd_prev = smoothed

            kinematic_applied = (
                _BASE_KINEMATIC_DRIVE
                and self._apply_kinematic_base_motion(direct_cmd, dt)
            )
            if kinematic_applied:
                # 指令値ではなく、衝突クランプ後に実現できた速度を返す。
                # ここは軌道追従の D 項 (odom_trajectory_action_server._velocity)
                # と次ステップの /odom twist の両方の元になるので、壁で止まった
                # ことがナビゲーション側から観測できるようになる。
                _kv = getattr(
                    self, '_base_kinematic_velocity', (0.0, 0.0, 0.0))
                abs_dot_x = float(_kv[0])
                abs_dot_y = float(_kv[1])
                cartesian_param_.dot_r = float(_kv[2])
                # ここで articulation root の速度を書いてはいけない。root は
                # FixedJoint で anchor に溶接されているので、拘束が要求する速度は
                # 常に anchor の速度そのもの。別の値を書くとソルバーが毎ステップ
                # それを打ち消し、その揺れが arm_flex まで伝わる。anchor と root の
                # 速度整合は、anchor を kinematic target で動かして PhysX 自身に
                # 速度を導出させることで成立させる
                # (BASE_KINEMATIC_ANCHOR_MODE=usd)。

            # 従来経路。決定論的更新が無効、または Tensor API が失敗した場合だけ
            # 剛体速度と位置ドライブを併用する。
            # BASE_VELOCITY_WRITE=0 にすると速度書き込みを省き、平面ジョイントの
            # ドライブ単独で走らせる (二重制御の切り分け用。既定は従来どおり)。
            # BASE_ROT_NO_VELOCITY_WRITE=1 は純回転のときだけ書き込みを省く。
            # 純回転では速度書き込みと位置ドライブの競合で並進が 3 倍に増える
            # (実測 0.31m -> 0.93m)。直進は速度書き込みが無いと成立しないため、
            # 回転のときだけ角度ドライブ単独にする。
            _pure_rotation = (
                _BASE_ROT_NO_VELOCITY_WRITE
                and abs(direct_cmd.dot_x) < 1e-3
                and abs(direct_cmd.dot_y) < 1e-3
                and abs(direct_cmd.dot_r) > 1e-3
            )
            if _BASE_DIAG and _BASE_ROT_NO_VELOCITY_WRITE:
                if _pure_rotation != getattr(self, '_pure_rot_prev', None):
                    self._pure_rot_toggle = (
                        getattr(self, '_pure_rot_toggle', 0) + 1)
                    print('[rot-switch] %s (%d回目) cmd=(%.4f, %.4f, %.4f)'
                          % ('純回転' if _pure_rotation else '通常',
                             self._pure_rot_toggle, direct_cmd.dot_x,
                             direct_cmd.dot_y, direct_cmd.dot_r), flush=True)
                    self._pure_rot_prev = _pure_rotation
            if (not kinematic_applied
                    and _BASE_VELOCITY_WRITE and not _pure_rotation):
                root_velocity = self.robots.get_velocities()[0]
                self.robots.set_velocities(np.array([[
                    direct_cmd.dot_x,
                    direct_cmd.dot_y,
                    float(root_velocity[2]),
                    0.0,
                    0.0,
                    direct_cmd.dot_r,
                ]], dtype=np.float32))
            self._base_direct_applied_cmd = direct_cmd
            # 平面ジョイントのドライブを「位置サーボ」として使う。目標は指令速度を
            # 積分した設定値 (setpoint)。現在位置に追従させるとバネが動きを
            # 打ち消してしまい走行できない (実測: 0.5m 指令で 0.12m しか進まず)。
            if not kinematic_applied:
                self._advance_base_setpoint(direct_cmd, dt)
            # ドライブの減衰項は速度に比例した抵抗になる (速度目標は PhysX に
            # 反映されないため実測で確認)。抵抗と釣り合う分だけ目標位置を
            # 先出しして打ち消す。lead = 減衰/剛性 x 指令速度。
            lead = (
                _BASE_BRAKE_C / _BASE_BRAKE_K if _BASE_BRAKE_K > 0.0 else 0.0)
            lead_ang = (
                _BASE_BRAKE_ANGULAR_C / _BASE_BRAKE_ANGULAR_K
                if _BASE_BRAKE_ANGULAR_K > 0.0 else 0.0)
            if not kinematic_applied:
                self._set_joint_brake_target((
                    self._base_setpoint[0] + lead * direct_cmd.dot_x,
                    self._base_setpoint[1] + lead * direct_cmd.dot_y,
                    self._base_setpoint[2] + lead_ang * direct_cmd.dot_r,
                ))
            if _BASE_DIAG and max(
                abs(direct_cmd.dot_x), abs(direct_cmd.dot_y),
                abs(direct_cmd.dot_r)) > 1e-3:
                self._base_move_diag_count = (
                    getattr(self, '_base_move_diag_count', 0) + 1)
                if self._base_move_diag_count % 20 == 1:
                    try:
                        _v = self.robots.get_velocities()[0]
                        _vx, _vy = float(_v[0]), float(_v[1])
                    except Exception:
                        _vx = _vy = 0.0
                    print(
                        '[move-diag] cmd=(%.3f, %.3f) 実速度=(%.3f, %.3f) '
                        'setpoint先行=(%.4f, %.4f)'
                        % (direct_cmd.dot_x, direct_cmd.dot_y, _vx, _vy,
                           self._base_setpoint[0]
                           - self.odometry_estimator.pose.x,
                           self._base_setpoint[1]
                           - self.odometry_estimator.pose.y),
                        flush=True,
                    )
            self.dc.set_dof_velocity_target(self.left_wheel_ptr, 0.0)
            self.dc.set_dof_velocity_target(self.right_wheel_ptr, 0.0)
            self.dc.set_dof_velocity_target(self.roll_ptr, 0.0)
        elif dt > 0.0:
            # MoveIt の仮想 odom 軌道は点列境界やFB補正で速度targetが小刻みに変わる。
            # そのまま PhysX の車輪/ステア velocity target へ入れると到達はしても
            # 加減速の角で車体がぐらつくため、実行直前に一次遅れ + 加速度制限を掛ける。
            # 停止時もこの経路を通す。以前は idle になった一フレームで速度目標を
            # 0へ落としていたため、強い車輪ドライブほど停止衝撃が大きくなっていた。
            _prev = getattr(self, '_base_cmd_prev', JointSpace())
            _alpha = min(1.0, dt / (_BASE_CMD_TAU + dt))

            # Apply the same acceleration scale to all three joint axes.
            # Independent clipping (previous implementation) changed the
            # wheel/steer ratio required by TwinCasterDrive, so a lateral
            # command first drove strongly in the wrong direction and shook
            # the chassis.  This is the same principle used by the real HSR
            # controller's coupled acceleration limiter.
            _filtered = JointSpace()
            _filtered.vel_wheel_l = _prev.vel_wheel_l + _alpha * (
                jcmd.vel_wheel_l - _prev.vel_wheel_l)
            _filtered.vel_wheel_r = _prev.vel_wheel_r + _alpha * (
                jcmd.vel_wheel_r - _prev.vel_wheel_r)
            _filtered.vel_steer = _prev.vel_steer + _alpha * (
                jcmd.vel_steer - _prev.vel_steer)
            _deltas_and_limits = (
                (_filtered.vel_wheel_l - _prev.vel_wheel_l,
                 _BASE_WHEEL_ACCEL_LIMIT),
                (_filtered.vel_wheel_r - _prev.vel_wheel_r,
                 _BASE_WHEEL_ACCEL_LIMIT),
                (_filtered.vel_steer - _prev.vel_steer,
                 _BASE_STEER_ACCEL_LIMIT),
            )
            _accel_scale = 1.0
            for _delta, _limit in _deltas_and_limits:
                if abs(_delta) > 0.0:
                    _accel_scale = min(
                        _accel_scale, _limit * dt / abs(_delta))
            _smoothed = JointSpace()
            _smoothed.vel_wheel_l = _prev.vel_wheel_l + _accel_scale * (
                _filtered.vel_wheel_l - _prev.vel_wheel_l)
            _smoothed.vel_wheel_r = _prev.vel_wheel_r + _accel_scale * (
                _filtered.vel_wheel_r - _prev.vel_wheel_r)
            _smoothed.vel_steer = _prev.vel_steer + _accel_scale * (
                _filtered.vel_steer - _prev.vel_steer)
            if _base_idle and max(
                abs(_smoothed.vel_wheel_l),
                abs(_smoothed.vel_wheel_r),
                abs(_smoothed.vel_steer),
            ) < 1e-3:
                _smoothed = JointSpace()
            jcmd = _smoothed
            self._base_cmd_prev = _smoothed

        if not _BASE_DIRECT_DRIVE:
            # Do not switch DOF stiffness at runtime.  Isaac Sim 4.5 rebuilds
            # the articulation for set_dof_properties(), invalidating cached
            # dynamic-control handles.
            self.dc.set_dof_velocity_target(
                self.left_wheel_ptr, jcmd.vel_wheel_l)
            self.dc.set_dof_velocity_target(
                self.right_wheel_ptr, jcmd.vel_wheel_r)
            self.dc.set_dof_velocity_target(self.roll_ptr, jcmd.vel_steer)

        force_readings = self.robots.get_measured_joint_forces(
            joint_indices=[
                self.robots._metadata.joint_indices['wrist_ft_sensor_frame_joint'] + 1]
        )
        # 物理ビュー/FTセンサが未準備のフレームでは None が返ることがある(特に world
        # 差し替え/reset 直後)。そのまま force_readings[0]... を実行すると毎フレーム
        # TypeError で step() 全体が落ち、以降の action server(腕/台車/グリッパの軌道実行)が
        # 一切動かず把持が進まない。ゼロ読みでフォールバックして step() を継続させる
        # (センサ復帰後は実値に戻る)。
        ft_valid = force_readings is not None
        if not ft_valid:
            force_readings = [[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]]
            if not getattr(self, '_ft_none_warned', False):
                self._ft_none_warned = True
                print('[hsr] get_measured_joint_forces returned None; FT/wrench をゼロで代用',
                      flush=True)
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_ros_time(
            self.simulation_context.current_time)
        wrench.header.frame_id = 'wrist_ft_sensor_frame'
        wrench.wrench.force.x = float(force_readings[0][0][0])
        wrench.wrench.force.y = float(force_readings[0][0][1])
        wrench.wrench.force.z = float(force_readings[0][0][2])
        wrench.wrench.torque.x = float(force_readings[0][0][3])
        wrench.wrench.torque.y = float(force_readings[0][0][4])
        wrench.wrench.torque.z = float(force_readings[0][0][5])
        self.ft_sensor_pub.publish(wrench)

        # gravity-compensated wrench: EMA ベースライン(重力)を差し引く。
        # 注意: None フォールバックの偽ゼロを baseline に取り込むと、ベースラインが0に
        # 初期化/引き寄せられ、実センサ復帰後しばらく補償後 wrench に重力ぶんが誤って残る
        # (接触検知を汚染する)。そのため初期化・EMA更新・comp配信は実FT読みのときだけ行う。
        if ft_valid:
            raw6 = [float(force_readings[0][0][i]) for i in range(6)]
            if not self._wrench_bias_inited:
                self._wrench_bias = list(raw6)
                self._wrench_bias_inited = True
            else:
                # alpha 小さめ: ゆっくりの重力変化は追従、接触の急変は残す。
                a = 0.02
                for i in range(6):
                    self._wrench_bias[i] += a * (raw6[i] - self._wrench_bias[i])
            comp = WrenchStamped()
            comp.header.stamp = wrench.header.stamp
            comp.header.frame_id = 'wrist_ft_sensor_frame'
            comp.wrench.force.x = raw6[0] - self._wrench_bias[0]
            comp.wrench.force.y = raw6[1] - self._wrench_bias[1]
            comp.wrench.force.z = raw6[2] - self._wrench_bias[2]
            comp.wrench.torque.x = raw6[3] - self._wrench_bias[3]
            comp.wrench.torque.y = raw6[4] - self._wrench_bias[4]
            comp.wrench.torque.z = raw6[5] - self._wrench_bias[5]
            self.ft_sensor_comp_pub.publish(comp)

        self.arm_trajectory_action_server.step(dt=dt)
        self.head_trajectory_action_server.step(dt=dt)
        self.odom_trajectory_action_server._velocity = (
            abs_dot_x,
            abs_dot_y,
            cartesian_param_.dot_r,
        )
        self.odom_trajectory_action_server.step(dt=dt)
        self.gripper_trajectory_action_server.step(dt=dt)
        self.gripper_apply_force_action_server.step(dt=dt)
        self.gripper_command_action_server.step(dt=dt)

        self.prev_time = self.simulation_context.current_time
