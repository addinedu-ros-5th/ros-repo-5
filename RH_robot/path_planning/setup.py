from setuptools import find_packages, setup

package_name = 'path_planning'

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
    maintainer='john',
    maintainer_email='yohangim119@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning_node = path_planning.path_planning_node:main',
            'robot_check = path_planning.ROS_communicate:main',
            'magnet_node = path_planning.magnet_node:main',
        ],
    },
)
