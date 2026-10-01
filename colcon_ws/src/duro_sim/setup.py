from setuptools import setup

package_name = 'duro_sim'

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
    description='Simulated SwiftNav Duro moving-baseline (dual-antenna RTK heading) publisher.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'baseline_node = duro_sim.baseline_node:main',
        ],
    },
)
