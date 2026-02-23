from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='sfuise').find('sfuise')
    
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'config_test_isas-walk3.yaml'
    ])
    
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
