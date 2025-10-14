from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    initial_pose_x = LaunchConfiguration('initial_pose.x')
    initial_pose_y = LaunchConfiguration('initial_pose.y')
    initial_pose_yaw = LaunchConfiguration('initial_pose.yaw')  # radians
    
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('initial_pose.x', default_value='0.0', description='Initial X position of the robot'),
        DeclareLaunchArgument('initial_pose.y', default_value='0.0', description='Initial Y position of the robot'),
        DeclareLaunchArgument('initial_pose.yaw', default_value='0.0', description='Initial yaw orientation of the robot in radians'),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'map_topic': '/map',
                'base_frame_id': 'base_footprint',   # or 'base_link' if that's your base
                'odom_frame_id': 'odom',
                'scan_topic': 'scan',
                # Initial pose params (yaw is in radians)
                'set_initial_pose': True,  
                'initial_pose.x': ParameterValue(initial_pose_x, value_type=float),
                'initial_pose.y': ParameterValue(initial_pose_y, value_type=float),
                'initial_pose.yaw': ParameterValue(initial_pose_yaw, value_type=float),
                'always_reset_initial_pose': True,
                # 'transform_tolerance': 0.5
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['amcl']
            }]
        )
    ])