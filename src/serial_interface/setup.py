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
('share/serial_interface/launch', ['launch/launch_serial.py']),
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
        'webcam_node = serial_interface.webcam_node:main',
        'ultrasonic_node = serial_interface.ultrasonic_node:main',
        'motor_driver = serial_interface.motor_driver:main',
        'motor_driver_node = serial_interface.motor_driver_node:main',
        'ultrasonic_sensor_node = serial_interface.ultrasonic_sensor_node:main',
        'serial_interface_node = serial_interface.serial_interface_node:main',
        'motion_planner_node = serial_interface.motion_planner_node:main',
    ],
},


)

