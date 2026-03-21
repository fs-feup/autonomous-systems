from setuptools import find_packages, setup

package_name = "invictasim_keys"

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
    maintainer="Your Name",
    maintainer_email="your.email@domain.com",
    description="Control mock for InvictaSim simulator via keyboard.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["invictasim_keys = invictasim_keys.main:main"],
    },
)
