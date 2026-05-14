# SPDX-License-Identifier: MIT

from setuptools import setup
from glob import glob
import os

package_name = "openamrobot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenAMRobot Maintainers",
    maintainer_email="botshare.ai@gmail.com",
    description="Top-level launch compositions for OpenAMRobot — wires together description, gazebo, nav2 and docking packages into reusable bringup launches.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
