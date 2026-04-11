import os
from glob import glob

from setuptools import find_packages, setup


package_name = "cororos2_navigation"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.lua"))),
        (os.path.join("share", package_name, "maps"), glob(os.path.join("maps", "*.pgm"))),
        (os.path.join("share", package_name, "maps"), glob(os.path.join("maps", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gabriela Vitez",
    maintainer_email="gabriela.vitez@b-robotized.com",
    description="Nav2, AMCL, and SLAM wrapper package for cororos2 robots.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_stamper = cororos2_navigation.cmd_vel_stamper:main",
            "scan_republisher = cororos2_navigation.scan_republisher:main",
        ],
    },
)
