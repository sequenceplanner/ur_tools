import os
from glob import glob
from setuptools import setup

package_name = "ur_setup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            os.path.join("share", package_name, "robots", "per"),
            glob("robots/per/*"),
        ),
        (
            os.path.join("share", package_name, "robots", "lage"),
            glob("robots/lage/*"),
        ),
        (
            os.path.join("share", package_name, "robots", "ursim"),
            glob("robots/ursim/*"),
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Endre Erős",
    author_email="endree@chalmers.se",
    maintainer="Endre Erős",
    maintainer_email="endree@chalmers.se",
    keywords=["ROS2"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="UR setup",
    license="no_license",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)