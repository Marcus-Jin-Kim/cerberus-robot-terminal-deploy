import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    pub_odom_tf = LaunchConfiguration('pub_odom_tf')
    initial_pose_x = LaunchConfiguration('initial_pose.x')
    initial_pose_y = LaunchConfiguration('initial_pose.y')
    initial_pose_yaw = LaunchConfiguration('initial_pose.yaw')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'CB_bringup_lidar.launch.py')
        ),
        launch_arguments={
            'namespace': ns,
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            'pub_odom_tf': pub_odom_tf
        }.items()
    )

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_nav'), 'launch', 'CB_acml_only_launch.py')
        ),
        launch_arguments={
            'namespace': ns,
            'initial_pose.x': initial_pose_x,
            'initial_pose.y': initial_pose_y,
            'initial_pose.yaw': initial_pose_yaw
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz'),
        DeclareLaunchArgument('rviz_config', default_value='bringup', description='RViz config'),
        DeclareLaunchArgument('pub_odom_tf', default_value='true', description='Publish odom->base TF'),
        DeclareLaunchArgument('initial_pose.x', default_value='0.0', description='AMCL initial X'),
        DeclareLaunchArgument('initial_pose.y', default_value='0.0', description='AMCL initial Y'),
        DeclareLaunchArgument('initial_pose.yaw', default_value='0.0', description='AMCL initial yaw (rad)'),
        bringup,
        amcl
    ])