import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'agv_pro_gazebo'
    pkg_dir = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_dir, 'urdf', 'agv_pro.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'square_cylinder_sphere.world')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'agvpro_display.rviz')

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}
    
    return LaunchDescription([

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # State publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                        '-entity', 'agv_pro'],
            output='screen'
        ),

        # Optional: RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),

        # --- New Nodes for TF2 and Octomap ---

        # TF2 Static Transform Publisher (map -> odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='dynamic_tf_publisher',
        #     name='dynamic_transform_publisher_map_odom',
        #     parameters=[{
        #         'parent_frame': 'map',
        #         'child_frame': 'odom',
        #         'translation': [0.0, 0.0, 0.0],
        #         'rotation': [0.0, 0.0, 0.0, 1.0],
        #         'publish_rate': 10.0
        #     }],
        # ),

        # # Octomap Server
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('octomap_server'), 'launch', 'octomap_tracking_server.launch.xml')
        #     ),
        #     # Set use_sim_time parameter for time synchronization with Gazebo
        #     launch_arguments={'use_sim_time': 'true'}.items()
        # ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('octomap_server'),
                    'launch',
                    'octomap_tracking_server.launch.xml'
                ])
            ])
        )
    ])