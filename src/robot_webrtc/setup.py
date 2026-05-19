from setuptools import find_packages, setup

package_name = 'robot_webrtc'

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
    maintainer='utsavchaudhary',
    maintainer_email='utsav@faunarobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'webrtc_node = robot_webrtc.webrtc_node:main',
            'signaling_server = robot_webrtc.signaling_server:main',
            'camera_manager_node = robot_webrtc.camera_manager_node:main',
            'webrtc_standalone_send = robot_webrtc.webrtc_standalone_send:main',
            'signaling_bridge = robot_webrtc.signaling_bridge:main',
            'stereo_camera_node = robot_webrtc.stereo_camera_node:main',
            'teleop_controller_node = robot_webrtc.teleop_controller_node:main',
            'operator_dashboard_node = robot_webrtc.operator_dashboard_node:main',
            'session_recorder_node = robot_webrtc.session_recorder_node:main',
        ],
    },
)
