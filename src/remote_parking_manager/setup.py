from setuptools import setup, find_packages

package_name = 'remote_parking_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jihyun',
    maintainer_email='you@example.com',
    description='Remote parking master FSM node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_manager = remote_parking_manager.mission_manager:main',
        ],
    },
)
