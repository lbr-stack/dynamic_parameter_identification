import os
from glob import glob

from setuptools import setup

package_name = "trajectory_acquisition"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.csv")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="martin.huber@kcl.ac.uk",
    description="Trajectory acquisition for position torque measurements",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_recording_node = trajectory_acquisition.joint_state_recording_node:main",
            "trajectory_execution_node = trajectory_acquisition.trajectory_execution_node:main",
        ],
    },
)
