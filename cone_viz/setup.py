from setuptools import setup

package_name = "cone_viz"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Visualize cones from perception and vehicle pose",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cone_marker_publisher = cone_viz.cone_marker_publisher:main",
        ],
    },
)
