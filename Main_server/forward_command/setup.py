from setuptools import find_packages, setup

package_name = 'forward_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='yohangim119@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_node = forward_command.forward_node:main',
            'waypoint_follower = forward_command.waypoint_follower:main',
            'task = forward_command.task:main',
        ],
    },
)
