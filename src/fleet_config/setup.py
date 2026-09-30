from setuptools import find_packages, setup

package_name = "fleet_config"

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
    description="Fleet definition rendered into the unit render tree",
    license="MIT",
    tests_require=["pytest"],
)
