"""Bring up the full drilling cell.

Everything site-specific (controller IP, camera serial numbers, detection
thresholds) lives in a parameter file — see kuka_eki/config/kuka_params.yaml.

    ros2 launch kuka_eki kuka_bringup.launch.py \
        params_file:=/path/to/my_params.yaml \
        cam1_serial:=_135122251049 cam2_serial:=_239622301497
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    kuka_eki_share = get_package_share_directory('kuka_eki')
    kuka_description_share = get_package_share_directory('kuka_description')
    realsense_share = get_package_share_directory('realsense2_camera')
    apriltag_share = get_package_share_directory('apriltag_ros')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(kuka_eki_share, 'config', 'kuka_params.yaml'),
            description='YAML file with all node parameters (IPs, ports, thresholds).',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(kuka_description_share, 'urdf', 'kuka.urdf.xacro'),
            description='Absolute path to the robot xacro/urdf.',
        ),
        DeclareLaunchArgument(
            'cam1_serial',
            default_value='_135122251049',
            description='Serial number of the end-effector RealSense (camera_1).',
        ),
        DeclareLaunchArgument(
            'cam2_serial',
            default_value='_239622301497',
            description='Serial number of the external/global RealSense (camera_2).',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', description='Start RViz.'
        ),
        DeclareLaunchArgument(
            'use_cameras', default_value='true',
            description='Start the RealSense drivers and AprilTag detection.',
        ),
        DeclareLaunchArgument(
            'home_on_start', default_value='true',
            description='Move to the home configuration before starting IK.',
        ),
    ]

    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_cameras = LaunchConfiguration('use_cameras')
    home_on_start = LaunchConfiguration('home_on_start')

    # ------------------------------------------------------------------
    # Robot description / visualisation
    # ------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('kuka_description'), 'rviz', 'display.rviz']
        )],
    )

    # ------------------------------------------------------------------
    # Cameras and fiducials
    #
    # NOTE: rs_launch.py expects 'camera_namespace', not 'namespace'. With
    # camera_namespace defaulting to 'camera', topics come out as
    # /camera/camera_1/... — which is what kuka_topic_mover subscribes to.
    # ------------------------------------------------------------------
    realsense_camera_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        condition=IfCondition(use_cameras),
        launch_arguments={
            'serial_no': LaunchConfiguration('cam1_serial'),
            'camera_name': 'camera_1',
            'camera_namespace': 'camera',
            'align_depth.enable': 'true',
        }.items(),
    )

    realsense_camera_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        condition=IfCondition(use_cameras),
        launch_arguments={
            'serial_no': LaunchConfiguration('cam2_serial'),
            'camera_name': 'camera_2',
            'camera_namespace': 'camera',
        }.items(),
    )

    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apriltag_share, 'launch', 'apriltag_real_detect.launch.py')
        ),
        condition=IfCondition(use_cameras),
    )

    # ------------------------------------------------------------------
    # Control chain
    # ------------------------------------------------------------------
    kuka_joint_server_node = Node(
        package='kuka_eki',
        executable='kuka_eki_joint_server',
        name='kuka_eki_joint_server',
        parameters=[params_file],
        output='screen',
    )

    home_pose_command = ExecuteProcess(
        condition=IfCondition(home_on_start),
        cmd=[
            'ros2', 'service', 'call', '/kuka_joint',
            'kuka_control_box_srvs/srv/KukaJoint',
            '{a1: 45.0, a2: -90.0, a3: 90.0, a4: 0.0, a5: 90.0, a6: -180.0}',
        ],
        output='screen',
    )

    def make_ik_node(condition=None):
        # kuka_ik_node declares no parameters of its own, so it is deliberately
        # not given params_file.
        return Node(
            package='kuka_control_box',
            executable='kuka_ik_node',
            name='kuka_ik_node',
            output='screen',
            condition=condition,
        )

    def make_plane_node(condition=None):
        return Node(
            package='kuka_task_client',
            executable='depth_plane_normal',
            name='depth_plane_normal',
            parameters=[params_file],
            output='screen',
            condition=condition,
        )

    drill_control_server_node = Node(
        package='kuka_eki',
        executable='drill_control_server',
        name='drill_control_server',
        parameters=[params_file],
        output='screen',
    )

    # Start IK and perception only once the home move has finished. This
    # replaces the previous fixed 3 s TimerAction, which could fire while the
    # robot was still moving.
    after_home = RegisterEventHandler(
        OnProcessExit(
            target_action=home_pose_command,
            on_exit=[make_ik_node(), make_plane_node()],
        ),
        condition=IfCondition(home_on_start),
    )

    # ...and start them immediately when homing is disabled, so the chain is
    # never left without an IK server.
    skip_home = UnlessCondition(home_on_start)

    return LaunchDescription(args + [
        robot_state_publisher_node,
        rviz_node,
        realsense_camera_1,
        realsense_camera_2,
        apriltag_launch,
        kuka_joint_server_node,
        home_pose_command,
        after_home,
        make_ik_node(skip_home),
        make_plane_node(skip_home),
        drill_control_server_node,
    ])
