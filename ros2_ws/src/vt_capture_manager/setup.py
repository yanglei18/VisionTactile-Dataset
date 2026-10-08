from glob import glob
from setuptools import find_packages, setup

setup(name="vt_capture_manager", version="0.5.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/vt_capture_manager"]),
                  ("share/vt_capture_manager", ["package.xml"]),
                  ("share/vt_capture_manager/launch", glob("launch/*.launch.py")),
                  ("share/vt_capture_manager/config", glob("config/*.yaml"))],
      install_requires=["setuptools", "PyYAML", "vt_capture_core"], zip_safe=True,
      maintainer="Vision Tactile Team", maintainer_email="vision-tactile@example.com",
      description="ROS capture snapshots and opt-in durable recorder management", license="Apache-2.0",
      entry_points={"console_scripts": ["vt_capture_manager = vt_capture_manager.node:main"]})
