from setuptools import find_packages, setup

package_name = 'server_and_gui_package'

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
            'my_first_node = server_and_gui_package.my_first_node:main',
            "server_and_user = server_and_gui_package.server_and_user:main",
            "come_in_park_area = server_and_gui_package.come_in_park_area:main",
            "server_and_park = server_and_gui_package.server_and_park:main"
        ],
    },
)
