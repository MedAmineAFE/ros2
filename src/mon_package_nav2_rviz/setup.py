from setuptools import setup
import os
from glob import glob

package_name = 'mon_package_nav2_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Inclure les fichiers de lancement
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi',
    maintainer_email='afeme46@gmail.com',
    description='Navigation project with IR sensor using Nav2 and RViz2',
    license='MIT',  # Remplace si tu as une autre licence
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_sensor_node = mon_package_nav2_rviz.ir_sensor_node:main',
        ],
    },
)

