from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = "vt_glove_bridge"
setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "LICENSE", "README.md"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Vision Tactile Team",
    maintainer_email="vision-tactile@example.com",
    description="Independent ROS 2 solved-pose bridge for the FSGlove GUI",
    license="Apache-2.0",
    license_files=["LICENSE"],
    tests_require=["pytest"],
    entry_points={"console_scripts": ["vt_glove_bridge_node = vt_glove_bridge.node:main"]},
)
