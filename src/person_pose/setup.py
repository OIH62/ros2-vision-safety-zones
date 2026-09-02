import os
from glob import glob
from setuptools import find_packages, setup
package_name = "person_pose"
setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OIH62",
    maintainer_email="qdsaxz05@gmail.com",
    description="YOLO pose publisher with automatic CUDA-to-CPU fallback.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["pose_publisher = person_pose.pose_publisher:main"]},
)
