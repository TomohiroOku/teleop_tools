from glob import glob

from setuptools import find_packages, setup

package_name = "teleop_tools"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tomohiro Oku",
    maintainer_email="tomohiro.oku@gmail.com",
    description="teleop tools for holonomic mobile robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["mouse_teleop = teleop_tools.mouse_teleop:main"],
    },
)
