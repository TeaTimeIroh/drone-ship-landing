from glob import glob

from setuptools import find_packages, setup

package_name = 'ship_landing'
share = 'share/' + package_name


def model_files(model):
    """Install entries for one Gazebo model folder."""
    return [
        (f'{share}/models/{model}', glob(f'models/{model}/model.*')),
        (f'{share}/models/{model}/materials/textures',
            glob(f'models/{model}/materials/textures/*.png')),
    ]


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
        *model_files('aruco_pad'),
        *model_files('moving_pad'),
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
        'console_scripts': [
            'takeoff_demo = ship_landing.takeoff_demo:main',
            'marker_detector = ship_landing.marker_detector:main',
            'land_on_pad = ship_landing.land_on_pad:main',
            'landing_trials = ship_landing.landing_trials:main',
            'pad_mover = ship_landing.pad_mover:main',
            'tilt_check = ship_landing.tilt_check:main',
            'nav_check = ship_landing.nav_check:main',
            'response_check = ship_landing.response_check:main',
            'stick_check = ship_landing.stick_check:main',
            'land_on_moving_pad = ship_landing.land_on_moving_pad:main',
            'moving_pad_trials = ship_landing.moving_pad_trials:main',
        ],
    },
)
