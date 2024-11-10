import os
import glob

from setuptools import find_packages, setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),


        ('share/' + package_name + '/launch', ['launch/launch.py',]),
        ('share/' + package_name + '/gazebo_models', ['gazebo_models/carModelURDF.urdf.xacro',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cobaltboy',
    maintainer_email='cobaltboy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'operatorNode = motor_controller.operatorNode:main'
        ],
    },
)
