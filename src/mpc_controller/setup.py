from setuptools import setup

package_name = "mpc_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mpc_controller.launch.py"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "casadi>=3.7.0",
        "acados_template>=0.5.1",
        "pyyaml",
    ],
    zip_safe=True,
    maintainer="Control Team",
    maintainer_email="control@example.com",
    description="MPC controller node (acados-based) for trajectory tracking.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_controller_node = mpc_controller.mpc_node:main",
        ],
    },
)
