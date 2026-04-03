from setuptools import setup

package_name = "roboclaw_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Gabriela Vitez",
    maintainer_email="gabriela.vitez@b-robotized.com",
    description="ROS 2 driver package for Roboclaw motor controllers.",
    license="Copyright (c) 2026, Proprietary License - b-robotized Group",
    entry_points={
        "console_scripts": [
            "roboclaw_driver_no_encoder = roboclaw_driver.roboclaw_no_encoder_node:main",
            "roboclaw_driver_encoder = roboclaw_driver.roboclaw_encoder_node:main",
        ],
    },
)
