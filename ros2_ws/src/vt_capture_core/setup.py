from setuptools import find_packages, setup


PACKAGE_NAME = "vt_capture_core"

setup(
    name=PACKAGE_NAME,
    version="0.6.0",
    packages=find_packages(exclude=["test", "test.*"]),
    package_data={PACKAGE_NAME: ["resources/*.xml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "LICENSE"]),
    ],
    python_requires=">=3.10",
    install_requires=[],
    extras_require={"test": ["pytest>=7"]},
    zip_safe=False,
    maintainer="Vision Tactile Team",
    maintainer_email="vision-tactile@example.com",
    description="ROS-independent capture workbench models, recipes and local protocol",
    license="Apache-2.0",
    license_files=["LICENSE"],
)
