from setuptools import setup

package_name = 'mujoco_pid_tuner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander James Becoy',
    maintainer_email='alexanderjames.becoy@outlook.com',
    description='Updates the PID/PD gains for the position/torque controllers in Mujoco simulation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_pid_tuner_node = mujoco_pid_tuner.mujoco_pid_tuner_node:main'
        ],
    },
)
