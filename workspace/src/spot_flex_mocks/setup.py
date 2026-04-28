from setuptools import find_packages, setup

package_name = 'spot_flex_mocks'

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
    description='Mock action/service servers for spot_flex demo development',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mock_nav_server = spot_flex_mocks.mock_nav_server:main',
            'mock_perception_server = spot_flex_mocks.mock_perception_server:main',
            'mock_policy_server = spot_flex_mocks.mock_policy_server:main',
            'mock_spot_services = spot_flex_mocks.mock_spot_services:main',
        ],
    },
)
