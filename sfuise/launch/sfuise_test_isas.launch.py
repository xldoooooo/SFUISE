from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='sfuise').find('sfuise')

    base_config = LaunchConfiguration('base_config')
    override_config = LaunchConfiguration('override_config')

    base_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        base_config,
    ])

    override_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        override_config,
    ])

    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'test.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_config',
            default_value='config_test_isas-base.yaml',
            description='Base parameter file under sfuise/config',
        ),
        DeclareLaunchArgument(
            'override_config',
            default_value='config_test_isas-base.yaml',
            description='Override parameter file under sfuise/config',
        ),
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
            parameters=[base_config_file, override_config_file],
            output='screen',
        ),
        Node(
            package='sfuise',
            executable='SplineFusion',
            name='SplineFusion',
            parameters=[base_config_file, override_config_file],
            output='screen',
        ),
    ])
