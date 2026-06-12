import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'invpendulum_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kingade',
    maintainer_email='kogodemilade@gmail.com',
    description='Inverted Pendulum controller package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = invpendulum_control.pid_controller:main',
            'lqr_controller = invpendulum_control.lqr_controller:main'
        ],
    },
)