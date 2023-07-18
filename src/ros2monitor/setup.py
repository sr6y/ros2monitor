from setuptools import setup

package_name = 'ros2monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer_email='sergioreymorales@outlook.es',
    description='ros2monitor: ROS2 Threat Detection Tool. More info README.md',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2monitor = ros2monitor.main:main'
        ],
    },
)
