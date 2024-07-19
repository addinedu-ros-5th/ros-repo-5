from setuptools import setup

package_name = 'aruco_marker_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Aruco marker detection and pose estimation using ROS 2',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_marker_detector_node = aruco_marker_detector.aruco_marker_detector_node:main'
        ],
    },
)
