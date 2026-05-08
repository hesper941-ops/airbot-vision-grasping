from setuptools import find_packages, setup

package_name = 'robot_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
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
            'camera_target_executor = robot_tasks.camera_target_executor:main',
            'grasp_task_open_loop = robot_tasks.grasp_task_open_loop:main',
            'grasp_task_visual_servo = robot_tasks.grasp_task_visual_servo:main',
            'visual_target_bridge = robot_tasks.visual_target_bridge:main',
        ],
    },
)
