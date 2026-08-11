from glob import glob
import os
from setuptools import find_packages, setup

package_name = "rover_collision_recovery"

setup(
    name=package_name,
    version="3.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samuel",
    maintainer_email="samuel@example.com",
    description=(
        "Collision Monitor-triggered recovery with local scan safety checks."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "collision_recovery_node = "
                "rover_collision_recovery.collision_recovery_node:main"
            ),
        ],
    },
)
