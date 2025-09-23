import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    return LaunchDescription([
        namespace_arg,
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'laser_scan_topic' : 'scan',
                'odom_topic' : 'odom_rf2o',
                'imu_topic' : 'imu/data',
                'publish_tf' : False,
                'base_frame_id' : 'base_footprint',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : '',
                'freq' : 20.0 # original DEFAULT is 20.0 
                # https://github.com/waveshareteam/ugv_ws/tree/ros2-humble-develop/src/ugv_else/robot_pose_publisher
            }],
            remappings=[
                    ('/tf', 'tf'),            # makes it /<ns>/tf
                    ('/tf_static', 'tf_static')
            ]
        )
    ])
    

            
            