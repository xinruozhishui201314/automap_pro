from setuptools import setup, find_packages
import os
from glob import glob

package_name = "overlap_transformer_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AutoMap-Pro",
    maintainer_email="automap@example.com",
    description="ROS2 service node for OverlapTransformer global descriptor",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "descriptor_server = overlap_transformer_ros2.descriptor_server:main",
        ],
    },
)
