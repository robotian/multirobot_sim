import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kinova_game_pad')
    default_config = os.path.join(pkg_share, 'config', 'cut_stem_gamepad.yaml')

    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('cut_stem_gamepad_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    cut_stem_gamepad_node = Node(
        package='kinova_game_pad',
        executable='cut_stem_gamepad_node',
        name='cut_stem_gamepad_node',
        namespace=namespace,
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='j100_0921',
            description='Robot namespace (joy topic and action are looked up under this namespace).',
        ),
        DeclareLaunchArgument(
            'cut_stem_gamepad_config',
            default_value=default_config,
            description='Path to the gamepad/CutStem parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation clock'
        ),
        cut_stem_gamepad_node,
    ])
