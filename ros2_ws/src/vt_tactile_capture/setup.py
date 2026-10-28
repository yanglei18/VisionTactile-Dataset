from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "vt_tactile_capture"

setup(
    name=PACKAGE_NAME,
    version="0.2.3",
    packages=find_packages(exclude=["test"]),
    package_data={PACKAGE_NAME: ["layouts/*.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "LICENSE", "README.md"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vision Tactile Team",
    maintainer_email="vision-tactile@example.com",
    description="Loss-aware tactile ADC capture and independent ROS heatmap viewer.",
    license="Apache-2.0",
    license_files=["LICENSE"],
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "vt_tactile_node = vt_tactile_capture.node:main",
        "vt_tactile_viewer = vt_tactile_capture.viewer:main",
        "vt_tactile_replay = vt_tactile_capture.replay:main",
        "vt_tactile_display_bridge = vt_tactile_capture.display_bridge:main",
    ]},
)
