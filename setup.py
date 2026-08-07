from setuptools import find_packages, setup

package_name = 'aruco_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/table_setup_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bastian',
    maintainer_email='bastianhunecke@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'world_pose_node = aruco_perception.world_pose_node:main',
            'detect_objects_node = aruco_perception.detect_objects_node:main'
        ],
    },
)
