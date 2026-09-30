import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path_context = setup_path.perform(context)

    # Read and parse robot YAML
    config = read_yaml(os.path.join(setup_path_context, 'robot.yaml'))
    clearpath_config = ClearpathConfig(config)
    namespace = clearpath_config.system.namespace

    # Robot Description (URDF)
    robot_description = {
        'robot_description': xacro.process_file(
            os.path.join(setup_path_context, 'robot.urdf.xacro')
        ).toxml()
    }

    # Semantic Robot Description (SRDF)
    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            os.path.join(setup_path_context, 'robot.srdf')
        ).toxml()
    }

    # Load Kinematics Configuration
    kinematics_yaml_path = os.path.join(setup_path_context, 'manipulators', 'config', 'moveit.yaml')
    with open(kinematics_yaml_path, "r") as f:
        full_moveit_config = yaml.safe_load(f)

    try:
        kinematics_dict = full_moveit_config[namespace]['move_group']['ros__parameters']['robot_description_kinematics']
    except KeyError:
        kinematics_dict = full_moveit_config.get('robot_description_kinematics', {})

    if "arm_0_gripper" in kinematics_dict:
        del kinematics_dict["arm_0_gripper"]

    robot_description_kinematics = {"robot_description_kinematics": kinematics_dict}

    # Shared defaults + per-robot overrides, the same files grid_cutter_action_server uses (they are keyed by that
    # node's name, so pick out just the parameters the stow node understands).
    pkg_share = get_package_share_directory('stow_arm_cpp')
    stow_keys = ('move_group', 'stow_pose', 'moveit_vel_scale', 'moveit_acc_scale', 'moveit_planning_time')
    stow_params = {}
    for name in ('grid_cutter_params.yaml', os.path.join('robots', f'{namespace}.yaml')):
        path = os.path.join(pkg_share, 'config', name)
        if os.path.isfile(path):
            with open(path, 'r') as f:
                params = yaml.safe_load(f)['/**/grid_cutter_action_server']['ros__parameters']
            stow_params.update({k: v for k, v in params.items() if k in stow_keys})

    # Your custom C++ Stow Node (Action Client)
    stow_node = Node(
        package='stow_arm_cpp',        
        executable='stow_arm_node',    
        name='stow_arm_node',          
        namespace=namespace,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            stow_params,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('joint_states', 'platform/joint_states'),
        ],
        output='screen'
    )

    return [stow_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup)
    ])