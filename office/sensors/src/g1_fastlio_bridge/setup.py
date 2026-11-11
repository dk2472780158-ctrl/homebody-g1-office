from glob import glob
from setuptools import setup

package_name = "g1_fastlio_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xu22",
    maintainer_email="xu22@example.com",
    description="G1 Mid360 preprocessing and launch helpers for FAST-LIO.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cloud_roll_pi_converter = g1_fastlio_bridge.cloud_roll_pi_converter:main",
            "imu_si_converter = g1_fastlio_bridge.imu_si_converter:main",
        ],
    },
)
