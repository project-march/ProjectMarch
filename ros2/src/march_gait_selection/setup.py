import os
from glob import glob
from setuptools import setup

package_name = 'march_gait_selection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'march_gait_selection.state_machine', 'march_gait_selection.dynamic_gaits'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'sounds'), glob('sounds/*.wav'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project March',
    maintainer_email='software@projectmarch.nl',
    description='Package that is responsible for receiving input from the input device, determining whether a request'
                'is valid and executing the gait',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'march_gait_selection = march_gait_selection.gait_selection_node:main',
            'test_console = march_gait_selection.gait_selection_node:test'
        ],
    },
)
