from setuptools import setup

package_name = "nexus_vision"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="NEXUS vision node",
    license="Apache-2.0",
    entry_points={"console_scripts": ["vision_node = nexus_vision.vision_node:main"]},
)
