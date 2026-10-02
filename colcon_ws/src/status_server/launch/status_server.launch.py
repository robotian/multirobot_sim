from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
    DeclareLaunchArgument('log_level', default_value='info', description='log level')
]


def generate_launch_description():
    LogInfo(msg="Launching nodes..."),
    remappings_tf=[
            ('/tf','tf'),
            ('/tf_static','tf_static'),
        ]
    
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')

    group_action = GroupAction(
        actions=[
            Node(
                package='status_server',
                executable='status_server',
                name='status_server',
                output='screen',
            ),
            # Task Manager Node
            Node(
                package='status_server',
                executable='task_manager',
                name='task_manager',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(group_action)

    return ld
