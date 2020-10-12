import os
from glob import glob, iglob

from setuptools import setup

package_name = 'march_shared_classes'


def data_files():
    data = [
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'test', 'resources'), ['test/resources/default.yaml']),
        (os.path.join('share', package_name, 'test', 'resources', 'walk'), ['test/resources/walk/walk.gait'])
    ]
    for file in iglob('test/resources/**/*.subgait', recursive=True):
        data.append((os.path.join('share', package_name, os.path.dirname(file)), [file]))
    return data


setup(
    name=package_name,
    version='0.0.0',
    packages=['src', 'src.gait', 'src.exceptions'],
    data_files=data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='katja',
    maintainer_email='katjaschmahl@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest', 'urfdom_py', 'unittest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
