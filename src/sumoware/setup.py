from setuptools import find_packages, setup

package_name = 'sumoware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sumoware_launch.py']),
    ],
    install_requires=['setuptools', "autoware_vehicle_msgs", "autoware_planning_msgs", "autoware_adapi_v1_msgs"],
    zip_safe=True,
    maintainer='frozturk',
    maintainer_email="faruk.oeztuerk@tum.de",
    description='Sumoware - Sumo Autoware Bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sumoware = sumoware.sumoware:main',
            'sumo_runner = sumo_runner.sumo_runner:main',
            'autoware_runner = autoware_runner.autoware_runner:main',
            'scenario_controller = scenario_controller.scenario_controller:main',
        ],
    },
)
