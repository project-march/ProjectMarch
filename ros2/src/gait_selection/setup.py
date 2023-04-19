from setuptools import setup

package_name = 'gait_selection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    scripts=['gait_selection/gait_loader.py'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Bak',
    maintainer_email='marco2771@gmail.com',
    description='Package that loads pre programmed gaits.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_selection_node = gait_selection.gait_selection_node:main'
        ],
    },
)
