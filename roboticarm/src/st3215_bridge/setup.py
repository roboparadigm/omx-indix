from setuptools import setup, find_packages

package_name = 'st3215_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',  # required by st3215 driver
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS 2 bridge for ST3215 servos',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'st3215_bridge_node = st3215_bridge.st3215_bridge_node:main',
        ],
    },
)
