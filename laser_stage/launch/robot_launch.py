import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    stage_ros2_dir = get_package_share_directory('stage_ros2')
    world_file = os.path.join(stage_ros2_dir, 'world', 'cave.world')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(stage_ros2_dir, 'launch', 'stage.launch.py')]),
            launch_arguments={'world': world_file}.items(),
        ),
        Node(
            package='laser_stage',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
            parameters=[{
                'initial_pose_x': -7.0,
                'initial_pose_y': -7.0,
                'initial_pose_a': 0.0,
            }],
        )
    ])
