from setuptools import setup

package_name = "nexus_sorting"

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
    description="NEXUS sorting node",
    license="Apache-2.0",
    entry_points={"console_scripts": ["sorting_node = nexus_sorting.sorting_node:main"]},
)
