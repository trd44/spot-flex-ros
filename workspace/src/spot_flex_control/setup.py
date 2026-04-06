from setuptools import find_packages, setup

package_name = 'spot_flex_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Timothy Duggan',
    maintainer_email='timothy.duggan@tufts.edu',
    description='Navigation, arm control, and policy execution for spot_flex',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nav_node = spot_flex_control.nav_node:main',
            'arm_node = spot_flex_control.arm_node:main',
            'policy_server_node = spot_flex_control.policy_server_node:main',
        ],
    },
)
