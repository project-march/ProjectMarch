from setuptools import setup
from glob import glob

package_name = 'march_test_joints_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/media', glob('media/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander James Becoy',
    maintainer_email='alexanderjames.becoy@projectmarch.nl',
    description='Utility package for testing joints in the MARCH exoskeleton using a GUI.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_joints_gui_node = march_test_joints_gui.test_joints_gui_node:main'
        ],
    },
)
