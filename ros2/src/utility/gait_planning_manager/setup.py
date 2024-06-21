from setuptools import setup

package_name = 'gait_planning_manager'

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
    maintainer='Femke Buiks',
    maintainer_email='femke.buiks@projectmarch.nl',
    description='This package manages the gait planning life cycle nodes.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_planning_manager_node = gait_planning_manager.gait_planning_manager_node:main'
        ],
    },
)
