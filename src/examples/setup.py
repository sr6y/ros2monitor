from setuptools import setup

package_name = 'examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    py_modules=[
        'examples.image_publisher_payload.image_publisher_payload',
        'examples.object_publisher_payload.object_publisher_payload'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer_email='sergioreymorales@outlook.es',
    description='Threats examples for ros2monitor',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher_payload = examples.image_publisher_payload.image_publisher_payload:main',
            'object_publisher_payload = examples.object_publisher_payload.object_publisher_payload:main'
        ],
    },
)
