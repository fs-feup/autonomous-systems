from setuptools import find_packages, setup

package_name = 'vehicle_dynamics_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bdias',
    maintainer_email='up201907828@up.pt',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "vehicle_dynamics_node = vehicle_dynamics_node.vehicle_dynamics_node:main",
            "integrator = vehicle_dynamics_node.integrator_node:main",
        ],
    },
)
