from setuptools import find_packages, setup

package_name = "person_zone_debug"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OIH62",
    maintainer_email="qdsaxz05@gmail.com",
    description="Lightweight RViz debug renderer for person zone topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "debug_node = person_zone_debug.debug_node:main",
            "status_monitor = person_zone_debug.status_monitor:main",
        ],
    },
)
