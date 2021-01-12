import os
from glob import iglob
from setuptools import setup, find_packages

package_name = "march_utility"


def data_files():
    data = [
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "test", "resources"),
            [os.path.join("test", "resources", "default.yaml")],
        ),
        (
            os.path.join("share", package_name, "test", "resources", "walk"),
            [os.path.join("test", "resources", "walk", "walk.gait")],
        ),
    ]
    for file in iglob(
        os.path.join("test", "resources", "**", "*.subgait"), recursive=True
    ):
        data.append(
            (os.path.join("share", package_name, os.path.dirname(file)), [file])
        )
    return data


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="Python and Cpp libraries used by other March packages",
    license="TODO: License declaration",
    tests_require=["pytest", "urfdom_py", "unittest", "parameterized"],
    entry_points={
        "console_scripts": [],
    },
)
