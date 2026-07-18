from setuptools import find_packages, setup

package_name = 'field_rover_sim'

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
    maintainer='Hamez Sheikh',
    maintainer_email='hamezys@gmail.com',
    description='Simulation, sensor generation, and visualization for the field rover.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'world_simulator = '
            'field_rover_sim.world_simulator_node:main',
            'range_sensor = '
            'field_rover_sim.range_sensor_node:main',
            'wheel_odometry = '
            'field_rover_sim.wheel_odometry_node:main',
            'imu_sensor = '
            'field_rover_sim.imu_sensor_node:main',
            'gps_sensor = '
            'field_rover_sim.gps_sensor_node:main',
        ],
    },
)
