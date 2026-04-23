from setuptools import setup
from glob import glob
import os

package_name = 'mavros_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='MAVROS examples',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'send_cmd_vel = mavros_examples.send_cmd_vel:main',
            'hover_test = mavros_examples.hover_test:main',
            'takeoff_and_wait = mavros_examples.takeoff_and_wait:main',
            'flight_test = mavros_examples.flight_test',
            'flight_test_v2 = mavros_examples.flight_test_v2:main',
            'challenge2or3 = mavros_examples.challenge2or3'
            
        ],
    },
)