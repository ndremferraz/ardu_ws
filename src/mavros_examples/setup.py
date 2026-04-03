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
            'flight_tasks_control = mavros_examples.flight_task_control:main',
            
        ],
    },
)