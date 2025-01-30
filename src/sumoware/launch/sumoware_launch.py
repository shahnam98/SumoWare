from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_lanelet
from crdesigner.common.config.lanelet2_config import lanelet2_config
import os
import subprocess


def _run_netconvert(input_file, output_file):
    if 'SUMO_HOME' not in os.environ:
        raise EnvironmentError("SUMO_HOME environment variable not set. Please set it to your SUMO installation directory.")

    command = [
        os.path.join(os.environ['SUMO_HOME'], 'bin', 'netconvert'),
        '--opendrive', input_file,
        '--offset.disable-normalization',
        '-o', output_file
    ]

    try:
        subprocess.run(command, check=True)
        print(f"Network converted successfully. Output saved to {output_file}")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred: {e}")

def _run_random_trips(input_file, output_path):
    if 'SUMO_HOME' not in os.environ:
        raise EnvironmentError("SUMO_HOME environment variable not set. Please set it to your SUMO installation directory.")

    command = [
        'python3',
        os.path.join(os.environ['SUMO_HOME'], 'tools', 'randomTrips.py'),
        '-n', input_file,
        '-r', Path.joinpath(output_path, 'sumo_map.rou.xml'),
        '-e', '120',
        '-l',
        '-o', Path.joinpath(output_path, 'sumo_map.trips.xml'),
        '--random',
        '--period', '1.6',
        '--fringe-factor', '10'
    ]

    try:
        subprocess.run(command, check=True)
        print(f"Random trips generated successfully. Output saved to {output_path}")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred: {e}")


def _convert_opendrive_to_lanelet(input_file: Path, output_file: Path):
    conf = lanelet2_config
    conf.autoware = True
    conf.use_local_coordinates = True
    opendrive_to_lanelet(input_file, output_file, lanelet2_config=conf)

def launch_setup(context, *args, **kwargs):
    map_file = LaunchConfiguration('map_file').perform(context)
    output_path = LaunchConfiguration('output_path').perform(context)
    project_absolute_path = LaunchConfiguration('project_absolute_path').perform(context)
    gpu_support = LaunchConfiguration('gpu_support').perform(context) == 'True'
    print(f"Map file: {map_file}")
    print(f"Output path: {output_path}")
    print(f"Project absolute path: {project_absolute_path}")
    print(f"GPU support: {gpu_support}")

    _convert_opendrive_to_lanelet(Path(map_file), Path(output_path + '/lanelet2_map.osm'))
    _run_netconvert(Path(map_file) , Path(output_path + '/sumo_map.net.xml'))
    _run_random_trips(Path(output_path + '/sumo_map.net.xml'), Path(output_path))

    return [
        Node(
            package='sumoware',
            executable='sumoware',
        ),
        Node(
            package='sumoware',
            executable='sumo_runner',
            parameters=[{'project_absolute_path': project_absolute_path}]
        ),
        Node(
            package='sumoware',
            executable='autoware_runner',
            parameters=[{'project_absolute_path': project_absolute_path, 'gpu_support': gpu_support}]
        ),
    ]

def generate_launch_description():
    input_path_arg = DeclareLaunchArgument(
        'input_path',
        default_value='',
        description='Path to the input file'
    )
    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value='',
        description='Path to the output file'
    )
    project_absolute_path_arg = DeclareLaunchArgument(
        'project_absolute_path',
        default_value='',
        description='Path to the project directory'
    )
    gpu_support_arg = DeclareLaunchArgument(
        'gpu_support',
        default_value="false",
        description='Whether to use GPU support'
    )

    return LaunchDescription([
        input_path_arg,
        output_path_arg,
        project_absolute_path_arg,
        gpu_support_arg,
        OpaqueFunction(function=launch_setup)
    ])