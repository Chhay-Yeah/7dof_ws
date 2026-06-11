from setuptools import find_packages, setup

package_name = 'arm_bot_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'src/*.so',
            'src/*.pyd',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='piranut',
    maintainer_email='piranut@local',
    description='Damiao CAN-FD motor driver for the 7-DOF arm '
                '(self-contained copy of tanerb_canfd_sub).',
    license='TODO',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'pos_motor_sub = arm_bot_hw.pos_motor_sub:main',
            'hw_bridge     = arm_bot_hw.hw_bridge:main',
            'damiao        = arm_bot_hw.damiao:main',
            'dev_sn        = arm_bot_hw.dev_sn:main',
        ],
    },
)
