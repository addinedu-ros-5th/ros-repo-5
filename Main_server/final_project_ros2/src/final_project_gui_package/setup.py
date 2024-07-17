from setuptools import find_packages, setup

package_name = 'final_project_gui_package'

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
    maintainer='',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_test_node = final_project_gui_package.gui_test_node:main',
            "from_gui_signal_subscriber = final_project_gui_package.from_gui_signal_subscriber:main",
            "from_ros2_signal_publisher = final_project_gui_package.from_ros2_signal_publisher:main",
            "out_signal_check_publisher = final_project_gui_package.out_signal_check_publisher:main"
        ],
    },
)
