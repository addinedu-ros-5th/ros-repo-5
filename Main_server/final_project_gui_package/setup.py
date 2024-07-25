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
            "send_park_num_test = final_project_gui_package.send_park_num_test:main",
            "server_and_user = final_project_gui_package.server_and_user:main",
            "out_signal_pre = final_project_gui_package.out_signal_pre:main",
            "in_park_area = final_project_gui_package.in_park_area:main"
        ],
    },
)
