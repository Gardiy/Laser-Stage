from setuptools import setup

package_name = 'laser_stage'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'laser_stage.robot_controller',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Garcia',
    maintainer_email='jgarcia@furg.br',
    description='A package for controlling a differential robot using ROS2 and Stage simulator.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = laser_stage.robot_controller:main',
        ],
    },
)
