import os
from glob import glob
from setuptools import setup


package_name = "ros2_vesc_drv"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vktork378",
    maintainer_email="viktor@vik.works",
    description="Provides basic VESC driver for differential drive app (2 VESCs)",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vesc_diff_drv=ros2_vesc_drv.vesc:main",
            "cmdv_mapper=ros2_vesc_drv.cmdv_mapper:main",
        ],
    },
)
