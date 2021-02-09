from setuptools import setup

package_name = "march_data_collector"


setup(
    name=package_name,
    version="0.0.0",
    packages=[
        package_name
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="Package that is responsible for computing the CoM using the IMU data "
                "and computing the capture point for balance purposes",
    license="TODO: License declaration",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "data_collector = march_data_collector.data_collector_node:main"
        ],
    },
)