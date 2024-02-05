import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ar_gripper"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.xml")),
        ),
        (
            os.path.join("share", package_name, "meshes"),
            glob(os.path.join("meshes", "*.stl")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.xacro")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexander Rössler",
    maintainer_email="alex@machinekoder.com",
    description="The ARgripper driver package",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ar_gripper = ar_gripper.scripts.ar_gripper:main",
            "ar_gripper_sim = ar_gripper.scripts.ar_gripper_sim:main",
            "ar_gripper_gui = ar_gripper.scripts.ar_gripper_gui:main",
        ],
    },
)
