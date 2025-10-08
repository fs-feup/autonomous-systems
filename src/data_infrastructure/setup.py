from setuptools import setup

package_name = 'data_infrastructure'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/data_infrastructure/launch', ['launch/data_infrastructure.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joana Louro',
    maintainer_email='joana.az.louro@gmail.com',
    description='ROS 2 node to publish and subscribe data.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_infrastructure_node = data_infrastructure.data_infrastructure_node:main',
        ],
    },
)