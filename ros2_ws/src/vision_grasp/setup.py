from setuptools import setup
import os
from glob import glob

package_name = 'vision_grasp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'bottle_detector_node = vision_grasp.bottle_detector_node:main',
            'grasp_coordinator_node = vision_grasp.grasp_coordinator_node:main',
            'hand_eye_calibration_node = vision_grasp.hand_eye_calibration_node:main',
        ],
    },
)
