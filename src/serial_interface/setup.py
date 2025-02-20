from setuptools import find_packages, setup

package_name = 'serial_interface'

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
    maintainer='rpi',
    maintainer_email='afeme46@gmail.com',
    description='ROS 2 package to interface with Arduino via serial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = serial_interface.serial_node:main',  
            'led_control_node = serial_interface.led_control_node:main',  
        ],
    },
)

