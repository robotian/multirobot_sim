from setuptools import setup

package_name = 'moveit_sim_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isaac_sim_project',
    maintainer_email='noreply@example.com',
    description=(
        'Bridges MoveIt trajectory/gripper execution to this project\'s own sim arm command topic, '
        'since the simulated arm has no real ros2_control hardware interface.'
    ),
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_sim_bridge = moveit_sim_bridge.bridge_node:main',
            # arm helpers, used by the web UI (tools/sim_ui) on sim and real robots alike
            'arm_goto = moveit_sim_bridge.arm_goto:main',
            'arm_joints = moveit_sim_bridge.arm_joints:main',
        ],
    },
)
