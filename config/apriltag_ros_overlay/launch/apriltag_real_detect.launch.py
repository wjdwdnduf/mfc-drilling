from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    apriltag_ros_prefix = get_package_share_directory('apriltag_ros')
    tags_global_yaml = os.path.join(apriltag_ros_prefix, 'cfg', 'tags_36h11_global.yaml')
    tags_local_yaml = os.path.join(apriltag_ros_prefix, 'cfg', 'tags_36h11_local.yaml')

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node_global',
            remappings=[
                ('/image_rect', '/camera/camera_2/color/image_raw'),
                ('/camera_info', '/camera/camera_2/color/camera_info')
            ],
            parameters=[{'use_sim_time': False}, tags_global_yaml]
        ),

        # End effector camera → 정밀 인식
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node_local',
            remappings=[
                ('/image_rect', '/camera/camera_1/color/image_raw'),
                ('/camera_info', '/camera/camera_1/color/camera_info')
            ],
            parameters=[{'use_sim_time': False}, tags_local_yaml]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
