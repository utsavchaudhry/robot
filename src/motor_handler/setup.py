import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'motor_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='had57',
    maintainer_email='had57@drexel.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_handler_node = motor_handler.motor_handler_node:main',
            'head_sinusoid_test = motor_handler.head_sinusoid_test:main',
            'motor_debug = motor_handler.motor_debug_node:main',
        ],
    },
)
