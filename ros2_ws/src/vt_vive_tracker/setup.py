from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "vt_vive_tracker"


setup(
    name=PACKAGE_NAME,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vision Tactile Team",
    maintainer_email="vision-tactile@example.com",
    description="Read-only ROS 2 publisher for VIVE Ultimate Tracker.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vt-vive-write-role-map = vt_vive_tracker.role_map_cli:main",
            "vt_vive_tracker_node = vt_vive_tracker.node:main",
            "vt_vive_tracker_visualizer = vt_vive_tracker.visualizer:main",
            "vt-vive-validate-topics = vt_vive_tracker.validate_topics:main",
        ],
    },
)
