import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='true',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )

    odom_publish_period_ms = DeclareLaunchArgument(
        'odom_publish_period_ms', default_value='25',
        description='Odom publish period in milliseconds'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to launch RViz2'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='bringup',
        description='Choose which rviz configuration to use'
    )

    # Include the robot state launch from the ugv_description package
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'CB_display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'namespace': LaunchConfiguration('namespace'),
        }.items()
    )

    # Define the nodes to be launched, now with namespace
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        namespace=LaunchConfiguration('namespace'),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
        namespace=LaunchConfiguration('namespace'),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Include laser lidar launch file, passing namespace if supported
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar'), 'launch', 'CB_ld19.launch.py') #'ldlidar.launch.py')
            # os.path.join(get_package_share_directory('ldlidar'), 'launch', 'ld19.launch.py') #'ldlidar.launch.py')
        ),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    # Include laser odometry launch file, passing namespace if supported
    rf2o_laser_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch', 'CB_rf2o_laser_odometry.launch.py')
            # os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py')
        ),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    # Define the base node with parameters and namespace
    # base_node = Node(
    #     package='ugv_base_node',
    #     executable='base_node',
    #     namespace=LaunchConfiguration('namespace'),
    #     parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    # )
    
    namespace = LaunchConfiguration('namespace')
    namespaced_odom_frame = PathJoinSubstitution([namespace, 'odom'])
    namespaced_base_footprint_frame = PathJoinSubstitution([namespace, 'base_footprint'])
    
    base_node = Node(
        package='ugv_base_node',
        executable='base_node',
        namespace=namespace,
        parameters=[
            # {'odom_frame': namespaced_odom_frame},
            #{'base_footprint_frame': namespaced_base_footprint_frame},
            # {'odom_frame': 'beast001/odom'}, # for testing
            # {'base_footprint_frame': 'beast001/base_footprint'},
            {'pub_odom_tf': LaunchConfiguration('pub_odom_tf')},
            {'odom_publish_period_ms': LaunchConfiguration('odom_publish_period_ms')}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Return the launch description with all defined actions
    return LaunchDescription([
        namespace_arg,
        pub_odom_tf_arg,
        use_rviz_arg,
        rviz_config_arg,
        odom_publish_period_ms,
        robot_state_launch, ## <- this is crucial for base_lidar_link!!
        bringup_node,
        driver_node,
        laser_bringup_launch,
        rf2o_laser_odometry_launch,
        base_node
    ])
