# SPDX-License-Identifier: MIT

from setuptools import setup
from glob import glob
import os

package_name = "openamrobot_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenAMRobot Maintainers",
    maintainer_email="botshare.ai@gmail.com",
    description="STUB. Reserved for perception modules beyond docking (see README).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
