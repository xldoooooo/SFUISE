from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package='sfuise').find('sfuise')
    
    # Configuration file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'config_test_isas-walk1.yaml'
    ])
    
    # RViz configuration file path
    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'test.rviz'
    ])

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'my_frame'],
            output='screen',
        ),
        Node(
            package='sfuise',
            executable='EstimationInterface',
            name='EstimationInterface',
            parameters=[config_file],
            output='screen',
        ),
        Node(
            package='sfuise',
            executable='SplineFusion',
            name='SplineFusion',
            parameters=[config_file],
            output='screen',
        ),
    ])
