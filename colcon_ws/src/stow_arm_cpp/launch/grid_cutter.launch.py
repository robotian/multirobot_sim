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

    config = read_yaml(os.path.join(setup_path_context, 'robot.yaml'))
    namespace = ClearpathConfig(config).system.namespace

    robot_description = {'robot_description': xacro.process_file(os.path.join(setup_path_context, 'robot.urdf.xacro')).toxml()}
    robot_description_semantic = {'robot_description_semantic': xacro.process_file(os.path.join(setup_path_context, 'robot.srdf')).toxml()}

    kinematics_yaml_path = os.path.join(setup_path_context, 'manipulators', 'config', 'moveit.yaml')
    with open(kinematics_yaml_path, "r") as f: full_moveit_config = yaml.safe_load(f)
    
    try: kinematics_dict = full_moveit_config[namespace]['move_group']['ros__parameters']['robot_description_kinematics']
    except KeyError: kinematics_dict = full_moveit_config.get('robot_description_kinematics', {})
    
    if "arm_0_gripper" in kinematics_dict: del kinematics_dict["arm_0_gripper"]

    pkg_share = get_package_share_directory('stow_arm_cpp')  # Replace with your actual package name
    param_file = os.path.join(pkg_share, 'config', 'grid_cutter_params.yaml')

    node_parameters = [
        robot_description,
        robot_description_semantic,
        {"robot_description_kinematics": kinematics_dict},
        {"use_sim_time": use_sim_time},
        param_file
    ]
    # Optional per-robot overrides (config/robots/<namespace>.yaml), applied after the shared defaults above: the
    # defaults are tuned for the Jackal j100_0921 (Gen3 Lite, 6 joints); robots with a different arm/gripper/mount
    # only list what differs. No file = the shared defaults, i.e. unchanged behaviour for j100_0921.
    robot_param_file = os.path.join(pkg_share, 'config', 'robots', f'{namespace}.yaml')
    if os.path.isfile(robot_param_file):
        node_parameters.append(robot_param_file)

    # The New Grid Cutter Node
    grid_cutter_node = Node(
        package='stow_arm_cpp',        
        executable='grid_cutter_action_server',    
        name='grid_cutter_action_server',          
        namespace=namespace,
        parameters=node_parameters,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('joint_states', 'platform/joint_states'),
        ],
        output='screen'
    )

    return [grid_cutter_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])