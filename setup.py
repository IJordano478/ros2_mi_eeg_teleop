from setuptools import find_packages, setup

package_name = 'ros2_mi_eeg_xyz_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uweai-ssd',
    maintainer_email='isaac_jordan@hotmail.co.uk',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_control = ros2_mi_eeg_xyz_control.joint_control:main',
            'calibrate_joint_waypoints = ros2_mi_eeg_xyz_control.calibrate_joint_waypoints:main',
            'xyz_control = ros2_mi_eeg_xyz_control.xyz_control:main'
        ],
    },
)
