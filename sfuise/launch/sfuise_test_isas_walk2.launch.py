from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='sfuise').find('sfuise')
    unified_launch = PathJoinSubstitution([
        pkg_share,
        'launch',
        'sfuise_test_isas.launch.py',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unified_launch),
            launch_arguments={
                'base_config': 'config_test_isas-base.yaml',
                'override_config': 'config_test_isas-base.yaml',
            }.items(),
        ),
    ])
