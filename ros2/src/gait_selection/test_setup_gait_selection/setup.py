from setuptools import setup

package_name = 'test_setup_gait_selection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    scripts=['test_setup_gait_selection/gait_loader.py'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco',
    maintainer_email='marco2771@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_setup_gait_selection_node = test_setup_gait_selection.test_setup_gait_selection_node:main'
        ],
    },
)
