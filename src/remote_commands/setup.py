from setuptools import setup

package_name = "remote_commands"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/phone.launch.py",
            "launch/ps3.launch.py",
            "launch/both.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="diogoalexandrepinheiro",
    maintainer_email="diogoalexandrepinheiro@gmail.com",
    description="Phone and PS3 controller teleop for normal car and pacsim.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "phone = remote_commands.phone:main",
            "ps3 = remote_commands.ps3:main",
        ],
    },
)
