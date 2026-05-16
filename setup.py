from setuptools import find_packages, setup

package_name = 'path_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/path_control.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/path_control.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Path smoothing and trajectory control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_smoother = path_control.path_smoother:main',
            'trajectory_generator = path_control.trajectory_generator:main',
            'trajectory_controller = path_control.trajectory_controller:main',
        ],
    },
)
