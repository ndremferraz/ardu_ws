from setuptools import find_packages, setup

package_name = 'pathfind_landing'

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
    maintainer='blub',
    maintainer_email='blub@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hybrid_landing = pathfind_landing.hybrid_landing:main',
            'landing = pathfind_landing.landing:main',
            'naive_landing = pathfind_landing.naive_landing:main',
        ],
    },
)
