from setuptools import find_packages, setup

package_name = 'crazyflie_traj'

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
    maintainer='aut',
    maintainer_email='filbe@dtu.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spiral_traj_node = crazyflie_traj.spiral_traj_node:main',
            'orchestrator = crazyflie_traj.orchestratorNode:main',
            'singleUAV = crazyflie_traj.singleUAVNode:main'
        ],
    },
)
