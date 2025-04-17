import os
from os import environ, pathsep
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def start_gzserver(context, *args, **kwargs):
    pkg_path = get_package_share_directory('pal_gazebo_worlds')
    priv_pkg_path = ''
    try:
        priv_pkg_path = get_package_share_directory('pal_gazebo_worlds_private')
    except Exception:
        pass

    world_name = LaunchConfiguration('world_name').perform(context)

    world = ''
    if os.path.exists(os.path.join(priv_pkg_path, 'worlds', world_name + '.world')):
        world = os.path.join(priv_pkg_path, 'worlds', world_name + '.world')
    elif os.path.exists(os.path.join(pkg_path, 'worlds', world_name + '.world')):
        world = os.path.join(pkg_path, 'worlds', world_name + '.world')

    params_file = PathJoinSubstitution(
        substitutions=[pkg_path, 'config', 'gazebo_params.yaml'])

    # Command to start the gazebo server.
    gazebo_server_cmd_line = [
        'gzserver', '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so', world,
        '--ros-args', '--params-file', params_file]
    # Start the server under the gdb framework.
    debug = LaunchConfiguration('debug').perform(context)
    if debug == 'True':
        gazebo_server_cmd_line = (
            ['xterm', '-e', 'gdb', '-ex', 'run', '--args'] +
            gazebo_server_cmd_line
        )

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=gazebo_server_cmd_line)

    return [start_gazebo_server_cmd]


def generate_launch_description():
    # Attempt to find pal_gazebo_worlds_private, use pal_gazebo_worlds otherwise
    try:
        priv_pkg_path = get_package_share_directory(
            'pal_gazebo_worlds_private')
        model_path = os.path.join(priv_pkg_path, 'models') + pathsep
        resource_path = priv_pkg_path + pathsep
    except Exception:
        model_path = ''
        resource_path = ''

    # Add pal_gazebo_worlds path
    pkg_path = get_package_share_directory('pal_gazebo_worlds')
    model_path += os.path.join(pkg_path, 'models')
    resource_path += pkg_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep+environ['GAZEBO_RESOURCE_PATH']

    # Add "headless" launch argument
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='true',
        description="Run Gazebo in headless mode (without GUI)"
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name', default_value='',
        description="Specify world name, we'll convert to full path"
    )

    declare_debug = DeclareLaunchArgument(
        'debug', default_value='False',
        choices=['True', 'False'],
        description='If debug start the gazebo world into a gdb session in an xterm terminal'
    )

    start_gazebo_server_cmd = OpaqueFunction(function=start_gzserver)

    # Only start gzclient if headless=False
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("headless"), "' == 'false'"]))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_headless)  #Add headless mode
    ld.add_action(declare_debug)
    ld.add_action(declare_world_name)

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))

    ld.add_action(start_gazebo_server_cmd)

    # Only add gzclient if not headless
    ld.add_action(start_gazebo_client_cmd)

    return ld
