from setuptools import find_packages, setup

package_name = 'sensor_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='test@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sensor_logger = sensor_logger.logger_node:main",
            "move_robot = sensor_logger.movement_node:main",
            "visualize_lidar = sensor_logger.visualizer_node:main"
        ],
    },
)
