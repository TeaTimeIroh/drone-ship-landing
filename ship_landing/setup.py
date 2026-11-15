from glob import glob

from setuptools import find_packages, setup

package_name = 'ship_landing'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share, ['package.xml']),
        (share + '/launch', glob('launch/*.launch.py')),
        (share + '/worlds', glob('worlds/*.sdf')),
        (share + '/config', glob('config/*.parm')),
        (share + '/models/aruco_pad', glob('models/aruco_pad/model.*')),
        (share + '/models/aruco_pad/materials/textures',
            glob('models/aruco_pad/materials/textures/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TeaTimeIroh',
    maintainer_email='th.tsantilas@gmail.com',
    description='Autonomous UAV precision landing on a moving vessel in GNSS-denied environments using ROS 2, Gazebo and ArduPilot.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['takeoff_demo = ship_landing.takeoff_demo:main'],
    },
)
