from setuptools import setup
from glob import glob

package_name = 'slam_benchmark'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josip Juric',
    maintainer_email='josip@todo.todo',
    description='SLAM Benchmark Suite for Bachelor Thesis',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tf_noise_injector = slam_benchmark.tf_noise_injector:main',
            'gt_publisher = slam_benchmark.gt_publisher:main',
            'trajectory_recorder = slam_benchmark.trajectory_recorder:main',
            'waypoint_navigator = slam_benchmark.waypoint_navigator:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
)
