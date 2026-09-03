#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import importlib.util

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='hsrb_description',
            description='Description package with robot URDF/xacro files.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='hsrb4s.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    # Isaac Sim 側 (この ros2 コンテナ) の MoveIt 付属 RViz2 を起動するか。
    # 既定 false: ふだん RViz は別 (Singularity 側等) で立てるので二重起動を避ける。
    # 立てたいときだけ `ros2 launch /hsr.launch.py use_rviz:=true`。
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch the MoveIt RViz2 on the Isaac Sim (ros2 container) side.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_grasp_tf',
            default_value='true',
            description='Broadcast /grasp_detector_node/result poses as TF frames for RViz.',
        )
    )
    # テレオペ "ロジック本体"(joystick_control + pseudo controllers)を Sim 側で起動するか。
    # 既定 true: 実機ではこれらがロボットオンボードで常時動くため、その代わりである Sim
    # 側で常時上げておく。前半(joy_node + teleop_steel_series)は Singularity 側 bringup の
    # 担当なのでここには含めない。ジョイスティック不要のタスク等で切りたいときだけ false。
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Launch joystick teleop logic (joystick_control + pseudo controllers).',
        )
    )
    # RGB-D の compressed/compressedDepth を作る中継ノードを起動するか。既定 true。
    # Isaac は raw の Image しか出さないが、実機の HSR は image_transport 経由で
    # /compressed・/compressedDepth も出す。消費側 (hma_pcl_reconst2 の use_compressed=true,
    # openmm/mmpose 等) はそちらを購読するので、sim でも同じ topic 構成を再現する。
    # これを false にすると /head_rgbd_sensor/reconsted/points が出なくなる。
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rgbd_republisher',
            default_value='true',
            description='Publish compressed RGB-D topics (pcl_reconst / openmm) via one republisher.',
        )
    )
    return declared_arguments


def render_xacro_and_launch_robot_state_publisher(context: LaunchContext, args: dict) -> str:
    robot_description_content = xacro.process_file(
        os.path.join(
            get_package_share_directory(
                context.perform_substitution(args['description_package'])),
            'robots',
            context.perform_substitution(args['description_file']),
        ),
        mappings={
            'gazebo_sim': 'False',
            'rviz_sim': 'False',
        },
    ).toxml()
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description_content},
            ],
            remappings=[('joint_states', '/whole_body/joint_states')],
            output={'both': 'log'},
        )
    ]


def launch_moveit_rviz(context: LaunchContext, args: dict):
    moveit_share = get_package_share_directory('hsrb_moveit_config')
    description_package = context.perform_substitution(
        args['description_package'])
    description_file = context.perform_substitution(args['description_file'])

    description_module_path = os.path.join(
        moveit_share, 'launch', 'robot_description.py')
    spec = importlib.util.spec_from_file_location(
        '_hsrb_moveit_robot_description', description_module_path)
    description_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(description_module)
    robot_description = {
        'robot_description': description_module.parse(
            description_package, description_file)
    }

    def load_text(relative_path):
        with open(os.path.join(moveit_share, relative_path), 'r') as stream:
            return stream.read()

    def load_yaml(relative_path):
        with open(os.path.join(moveit_share, relative_path), 'r') as stream:
            return yaml.safe_load(stream)

    robot_description_semantic = {
        'robot_description_semantic': load_text('config/hsrb.srdf')
    }
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join([
                'default_planner_request_adapters/AddTimeOptimalParameterization',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints',
            ]),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(
        load_yaml('config/ompl_planning.yaml'))

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                load_yaml('config/kinematics.yaml'),
                {'use_sim_time': True},
            ],
            arguments=[
                '-d', os.path.join(moveit_share, 'config', 'moveit.rviz')
            ],
            respawn=True,
            respawn_delay=2.0,
            additional_env={
                '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
                'QT_X11_NO_MITSHM': '1',
            },
        )
    ]


def generate_launch_description():
    args = {}
    for arg in declare_arguments():
        args[arg.name] = LaunchConfiguration(arg.name)

    relay_node = GroupAction(
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource([
                    'hsrb_relay_topics.launch.xml',
                ]),
            )
        ]
    )

    sensor_frames = GroupAction(
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource([
                    'hsrb_sensor_frames.launch.xml',
                ]),
            )
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {'source_list': ['/joint_states']},
        ],
        namespace='whole_body',
        remappings=[('robot_description', '/robot_description')],
    )

    robot_state_publisher = OpaqueFunction(
        function=render_xacro_and_launch_robot_state_publisher, args=[args]
    )

    common = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('hsrb_common_launch'),
                        'launch',
                        'hsrb_common.launch.py',
                    )
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    # false: HSR 純正 localizer/nav (laser_2d_localizer, pose_integrator)
                    # を起動しない。自己位置推定は Singularity 側の emcl2/pumas に任せる
                    # (本番同等)。robot_state_publisher / odom / 知覚は下で別途上げるので残る。
                    # HSR 純正ナビを使いたいときだけ 'true' に戻す。
                    'use_navigation': 'false',
                    'map': os.path.join(
                        get_package_share_directory('tmc_wrs_gazebo_worlds'),
                        'maps',
                        'wrs2020',
                        'map.yaml',
                    ),
                    'use_manipulation': 'true',
                    'use_teleop': 'false',
                    'use_joy_node': 'false',
                }.items(),
            ),
        ]
    )

    moveit = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('hsrb_moveit_config'),
                        'launch',
                        'hsrb_demo.launch.py',
                    )
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    # RViz is launched below with respawn enabled so a transient
                    # GL initialization failure cannot remove the planning UI.
                    'use_rviz': 'false',
                }.items(),
            ),
        ]
    )

    moveit_rviz = OpaqueFunction(
        function=launch_moveit_rviz,
        args=[args],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # task_evaluators = GroupAction(
    #    actions=[
    #        IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource(
    #                [
    #                    os.path.join(
    #                        get_package_share_directory('tmc_gazebo_task_evaluators'),
    #                        'launch',
    #                        'robocup2021.launch.py',
    #                    )
    #                ]
    #            ),
    #            launch_arguments={
    #                'use_sim_time': 'true'
    #            }.items(),
    #        ),
    #    ]
    # )

    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(
                'hsrb_bringup'), 'launch', 'odoms.py')
        ])
    )

    # テレオペ "ロジック本体"。同じ階層の teleop.launch.py を参照。
    # (ros2 コンテナでは /hsr.launch.py と /teleop.launch.py が並んで配置される)
    teleop = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(os.path.dirname(
                        os.path.abspath(__file__)), 'teleop.launch.py')
                ]),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_teleop')),
    )

    launch_dir = os.path.dirname(os.path.abspath(__file__))

    # RGB-D の compressed/compressedDepth を作る中継ノード。入力(生Image)→出力
    # (CompressedImage) のマッピングはスクリプト側 DEFAULT_MAPPINGS に集約してあり、
    # 実在する入力だけが流れる(無い入力は無出力)。
    local_rgbd_republisher = os.path.join(
        os.path.dirname(launch_dir),
        'scripts',
        'rgbd_republisher.py',
    )
    rgbd_republisher_script = (
        local_rgbd_republisher
        if os.path.exists(local_rgbd_republisher)
        else '/rgbd_republisher.py'
    )
    rgbd_republisher = ExecuteProcess(
        cmd=[
            'python3',
            rgbd_republisher_script,
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ],
        output='screen',
        condition=IfCondition(args['use_rgbd_republisher']),
    )

    laser_scan_matcher = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        remappings=[
            ('odom', 'laser_odom'),
            ('scan', 'scan'),
        ],
        parameters=[
            {
                'laser_frame': 'base_range_sensor_link',
                'publish_odom': 'laser_odom',
                'use_sim_time': True,
            }
        ],
    )

    local_matcher_helper = os.path.join(
        os.path.dirname(launch_dir),
        'scripts',
        'reset_world_matcher_helper.py',
    )
    matcher_helper_script = (
        local_matcher_helper
        if os.path.exists(local_matcher_helper)
        else '/reset_world_matcher_helper.py'
    )
    reset_world_matcher_helper = ExecuteProcess(
        cmd=[
            'python3',
            matcher_helper_script,
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ],
        output='screen',
        additional_env={'HSR_ROS_VERSION': '2'},
    )

    grasp_tf_broadcaster = ExecuteProcess(
        cmd=[
            'python3',
            '/examples/grasp_tf_broadcaster.py',
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_grasp_tf')),
    )

    # Isaac の真値odom(hsr.py で wheel_odom を物理真値で上書き済み)を TF/計画に使うため、
    # 起動後に odometry_switcher を wheel_odom へ切り替える。既定の laser_odom は壁の少ない
    # 疎な環境でスキャンマッチングが破綻し odom が飛ぶため、真値の wheel を使う。
    # サービスが上がるまでリトライ。※ sim 専用(hsr.launch.py=Isaac用)。実機の odoms は laser のまま。
    switch_odom_to_wheel = ExecuteProcess(
        cmd=['bash', '-c',
             'for i in $(seq 1 60); do '
             'if ros2 service call /odometry_switch '
             'tmc_navigation_msgs/srv/OdometrySwitch '
             '"{odom_type: {data: wheel_odom}}" 2>/dev/null '
             '| grep -q "is_success=True"; then '
             'echo "[odom] switched to wheel_odom (=Isaac gt odom)"; exit 0; fi; '
             'sleep 2; done; '
             'echo "[odom] WARN: failed to switch odometry to wheel_odom"'],
        output='screen',
    )

    nodes = [
        relay_node,
        rgbd_republisher,
        laser_scan_matcher,
        reset_world_matcher_helper,
        sensor_frames,
        joint_state_publisher,
        robot_state_publisher,
        common,
        moveit,
        moveit_rviz,
        grasp_tf_broadcaster,
        odom,
        switch_odom_to_wheel,
        teleop,
        # task_evaluators
    ]

    return LaunchDescription(
        declare_arguments()
        + [
            SetParameter(name='use_sim_time', value=True),
        ]
        + nodes
    )
