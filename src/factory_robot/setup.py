from setuptools import setup, find_packages
package_name = 'factory_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['factory_robot', 'factory_robot.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/maps', ['maps/factory1.txt', 'maps/factory2.txt', 'maps/factory3.txt', 'maps/factory4.txt']),
        (f'share/{package_name}/launch', ['launch/factory.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diogo',
    maintainer_email='diogoalexandrepinheiro@gmail.com',
    description='Factory robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'factory_node = factory_robot.nodes.factory_node:main',
            'robot_keys = factory_robot.nodes.robot_keys:main',  # <-- REQUIRED
        ],
    },
)
