from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'march_rqt_input_device'

setup(
    name=package_name,
    version='0.0.0',
    packages=['src'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['plugin.xml']),
        (os.path.join('share', package_name), ['resource/input_device.ui']),
        (os.path.join('share', package_name, 'resource', 'img'), glob('resource/img/*.png'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project March',
    maintainer_email='katjaschmahl@hotmail.com',
    description='Developer input device to send commands to the march exoskeleton',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_device = src.input_device_plugin:main'
        ],
    },
)
