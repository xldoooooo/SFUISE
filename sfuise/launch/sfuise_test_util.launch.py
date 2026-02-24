from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='sfuise').find('sfuise')
    anchor_path = LaunchConfiguration('anchor_path')
    
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'config_test_util.yaml'
    ])
    
    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'test.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'anchor_path',
            default_value=EnvironmentVariable(
                'SFUISE_UTIL_ANCHOR_PATH',
                default_value='/path/to/UTIL/dataset/util-uwb-dataset-main/dataset/flight-dataset/survey-results/anchor_const1_survey.txt'
            ),
            description='Path to UTIL anchor survey file'
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
            parameters=[config_file, {'anchor_path': anchor_path}],
            output='screen',
        ),
        Node(
            package='sfuise',
            executable='SplineFusion',
            name='SplineFusion',
            parameters=[config_file, {'anchor_path': anchor_path}],
            output='screen',
        ),
    ])
