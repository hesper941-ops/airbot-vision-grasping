from setuptools import setup
import os, glob

package_name = 'emotion_local'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xjt',
    maintainer_email='xjt@example.com',
    description='5-class face emotion recognition on RDK X5 BPU',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'emotion_node = emotion_local.emotion_node:main',
            'emotion_fusion = emotion_local.emotion_fusion_node:main',
        ],
    },
)
