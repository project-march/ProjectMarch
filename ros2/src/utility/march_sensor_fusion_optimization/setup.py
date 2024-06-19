from setuptools import setup

package_name = 'march_sensor_fusion_optimization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor_fusion_optimization.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander James Becoy',
    maintainer_email='alexanderjames.becoy@projectmarch.nl',
    description='This package optimizes the sensor fusion in march state estimator via noise parameter tuning.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion_optimizer_node = march_sensor_fusion_optimization.sensor_fusion_optimizer_node:main',
        ],
    },
)
