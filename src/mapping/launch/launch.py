import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Clearpath GZ Simülasyonunu Dahil Etme
    clearpath_gz_share = FindPackageShare(package='clearpath_gz').find('clearpath_gz')
    simulation_launch_path = os.path.join(clearpath_gz_share, 'launch', 'simulation.launch.py')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={
            'x': '-3.3',
            'y': '3.3',
            'world': '/home/furkan/Documents/map/worlds/generated_maze',
            'rviz': 'true',
            'use_sim_time': 'true'
        }.items()
    )

    # 2. Localization Node
    localization_node = Node(
        package='mapping',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )

    mapping_node = Node(
        package='mapping',
        executable='mapping_node',
        name='mapping_node',
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )

    return LaunchDescription([
        simulation_launch,
        localization_node,
        mapping_node 
    ])