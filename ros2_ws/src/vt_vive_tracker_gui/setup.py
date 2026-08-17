from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "vt_vive_tracker_gui"


setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml", *glob("README*")]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vision Tactile Team",
    maintainer_email="vision-tactile@example.com",
    description="Standalone GUI for VIVE Ultimate Tracker visualization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vt_vive_tracker_gui = vt_vive_tracker_gui.main:main",
        ]
    },
)
