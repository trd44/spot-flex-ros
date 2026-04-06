from setuptools import find_packages, setup

package_name = 'spot_flex_ui'

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
    description='User interface for spot_flex task commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ui_node = spot_flex_ui.ui_node:main',
        ],
    },
)
