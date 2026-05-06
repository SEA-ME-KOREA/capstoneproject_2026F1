from setuptools import find_packages, setup

package_name = 'my_valet_parking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    scripts=[
        'scripts/limo_lane_mission.py',
        'scripts/parking_detector.py',
        'scripts/limo_parking_planner.py',
        'scripts/simple_straight_mission.py',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['rviz/parking_status.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leesumyeong',
    maintainer_email='leesumyeong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'limo_lane_mission = my_valet_parking.limo_lane_mission:main',
            'limo_evasion_controller = my_valet_parking.limo_evasion_controller:main',
            'limo_f1tenth_pure_pursuit = my_valet_parking.limo_f1tenth_pure_pursuit:main',
        ],
    },
)
