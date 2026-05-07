from setuptools import find_packages, setup

package_name = 'robot_arm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'state_node = robot_arm_driver.state_node:main',
        'move_node = robot_arm_driver.move_node:main',
        'gripper_node = robot_arm_driver.gripper_node:main',
        'init_arm = robot_arm_driver.init_arm:main',
        'sleep_arm = robot_arm_driver.sleep_arm:main',
        'end_position_publisher = robot_arm_driver.end_position_publisher:main',
        ],
    },
)
