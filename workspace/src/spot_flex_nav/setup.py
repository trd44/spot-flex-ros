from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'spot_flex_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'locations'), glob('locations/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Timothy Duggan',
    maintainer_email='timothy.duggan@tufts.edu',
    description='Camera-based Nav2 integration, mapping, and named locations for Spot Flex.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'depth_to_scan = spot_flex_nav.depth_to_scan_node:main',
            'multi_depth_to_scan = spot_flex_nav.multi_depth_to_scan_node:main',
            'odom_to_tf = spot_flex_nav.odom_to_tf_node:main',
            'teleop_arrows = spot_flex_nav.teleop_arrows:main',
            'tag_location = spot_flex_nav.tag_location:main',
            'tag_graphnav_location = spot_flex_nav.tag_graphnav_location:main',
            'go_to_location = spot_flex_nav.go_to_location:main',
        ],
    },
)
