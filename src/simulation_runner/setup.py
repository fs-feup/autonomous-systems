from setuptools import find_packages, setup

package_name = "simulation_runner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Diogo Pinheiro",
    maintainer_email="diogoleandrogpt@gmail.com",
    description=(
        "Runs the InvictaSim full stack through the configured scenarios and checks the "
        "resulting driving performance against their criteria."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run = simulation_runner.cli:main",
        ],
    },
)
