from setuptools import find_packages, setup

package_name = 'vehicle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/vehicle_control/worlds', ['worlds/diff_robot.sdf']),
        ('share/vehicle_control/launch', ['launch/bridge.launch.py']),
        ('share/vehicle_control/launch', ['launch/world.launch.py']),
        ('share/vehicle_control/launch', ['launch/controller.launch.py']),
        ('share/vehicle_control/launch', ['launch/lidar_listener.launch.py']),
        ('share/vehicle_control/launch', ['launch/vehicle_sim.launch.py']),        
        ('share/vehicle_control/launch', ['launch/mpc_controller.launch.py']),        
        ('share/vehicle_control/launch', ['launch/casadi_mpc_controller.launch.py']),
        ('share/vehicle_control/config', ['config/casadi_mpc_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustavo_admin',
    maintainer_email='g.olivas.m@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_controller = vehicle_control.vehicle_controller:main',
            'lidar_listener = vehicle_control.lidar_listener:main',
            'mpc_controller = vehicle_control.mpc_controller:main',
            'casadi_mpc_controller = vehicle_control.casadi_mpc_controller:main',
        ],
    },
)
