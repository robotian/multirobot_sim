from glob import glob
from setuptools import find_packages, setup

package_name = 'status_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        # ('share/' + package_name + '/data', glob('data/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vupadhye',
    maintainer_email='vupadhye@mtu.edu',
    description='Node to subscribe status monitor on each husky',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_server = status_server.robot_status_sync:main',
            'task_manager = status_server.job_publisher:main',
        ],
    },
)
