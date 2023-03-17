from setuptools import setup
import os
from glob import glob

package_name = 'ros2_hgr'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/hgr.launch.xml']),
        (os.path.join('share', package_name, 'model/keypoint_classifier'), glob('model/keypoint_classifier/*')),
        (os.path.join('share', package_name, 'model/point_history_classifier'), glob('model/point_history_classifier/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avaz',
    maintainer_email='AvaZahedi2023@u.northwestern.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hgr_node = ros2_hgr.hgr_node:main",
            "hgr_node_cam = ros2_hgr.hgr_node_cam:main"
        ],
    },
)
