import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    pub_odom_tf = LaunchConfiguration('pub_odom_tf')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    domain_bridge_config_file_fullpath = LaunchConfiguration('domain_bridge_config_file_fullpath')

    # bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'CB_bringup_lidar.launch.py')
    #     ),
    #     launch_arguments={
    #         'namespace': ns,
    #         'use_rviz': use_rviz,
    #         'rviz_config': rviz_config,
    #         'pub_odom_tf': pub_odom_tf
    #     }.items()
    # )

    # amcl = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ugv_nav'), 'launch', 'CB_acml_only_launch.py')
    #     ),
    #     launch_arguments={
    #         'namespace': ns,
    #         'initial_pose_x': initial_pose_x,
    #         'initial_pose_y': initial_pose_y,
    #         'initial_pose_yaw': initial_pose_yaw
    #     }.items()
    # )

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_nav'), 'launch', 
            'CB_launch_cartographer_localization_only.py')
        ),
        launch_arguments={
            'namespace': ns,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw
        }.items()
    )

    domain_bridge = Node(
        package='domain_bridge', executable='domain_bridge',
        name='domain_bridge',
        output='screen',
        arguments=[
            domain_bridge_config_file_fullpath
        ],
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static')
        # ]
    )

    # Node(
    #     package='domain_bridge', executable='


    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz'),
        DeclareLaunchArgument('rviz_config', default_value='bringup', description='RViz config'),
        DeclareLaunchArgument('pub_odom_tf', default_value='true', description='Publish odom->base TF'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0', description='AMCL initial X'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0', description='AMCL initial Y'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0', description='AMCL initial yaw (rad)'),
        DeclareLaunchArgument('domain_bridge_config_file_fullpath', description='Full path to domain bridge config file'),
        # bringup,
        cartographer,
        domain_bridge
    ])