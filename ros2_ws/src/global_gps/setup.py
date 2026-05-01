import os
from glob import glob

from setuptools import setup

package_name = "global_gps"

setup(
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), ["launch/global_gps.launch.py"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
)
