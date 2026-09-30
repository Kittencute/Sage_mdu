from setuptools import find_packages, setup

package_name = "fleet_unit"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emil Persson",
    maintainer_email="emil.persson@mdu.se",
    description="Manifest-driven runtime for one robot unit",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_unit = fleet_unit.cli:main",
            "unit_agent = fleet_unit.unit_agent:main",
        ],
    },
)
