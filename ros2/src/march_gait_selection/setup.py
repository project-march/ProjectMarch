import os
from glob import glob, iglob
from setuptools import setup

package_name = 'march_gait_selection'

def data_files():
    data = [
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'test', 'resources'),
         [os.path.join('test', 'resources', 'default.yaml')]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ]
    for file in iglob(os.path.join('test', 'resources', '**', '*.subgait'), recursive=True):
        data.append((os.path.join('share', package_name, os.path.dirname(file)), [file]))
    for file in iglob(os.path.join('test', 'resources', '**', '*.gait'), recursive=True):
        data.append((os.path.join('share', package_name, os.path.dirname(file)), [file]))
    return data



setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'march_gait_selection.state_machine', 'march_gait_selection.dynamic_gaits'],
    data_files=data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project March',
    maintainer_email='software@projectmarch.nl',
    description='Package that is responsible for receiving input from the input device, '
                'determining whether a request is valid and executing the gait',
    license='TODO: License declaration',
    tests_require=['pytest', 'unittest'],
    entry_points={
        'console_scripts': [
            'march_gait_selection = march_gait_selection.gait_selection_node:main'
        ],
    },
)
