from setuptools import setup

package_name = 'march_aie_gait_planning'

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
    maintainer='Alexander James Becoy, Martijn Habers',
    maintainer_email='alexanderjames.becoy@projectmarch.nl. martijn.habers@projectmarch.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'march_aie_gait_planning_node = march_aie_gait_planning.march_aie_gait_planning_node:main',
            'one_leg_balance_node = march_aie_gait_planning.one_leg_balance_node:main',
            'two_legs_weightshift_node = march_aie_gait_planning.two_legs_weightshift_node:main',
            'keyframe_node = march_aie_gait_planning.keyframe_node:main',
        ],
    },
)
