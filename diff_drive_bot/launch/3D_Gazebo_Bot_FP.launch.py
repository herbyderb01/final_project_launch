from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Define paths to required files
    diff_drive_bot_pkg_path = get_package_share_directory('diff_drive_bot')
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')

    # Paths to individual launch files
    rsp_launch_path = os.path.join(diff_drive_bot_pkg_path, 'launch', 'rsp.launch.py')
    gz_sim_launch_path = os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
    gz_spawn_model_path = os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_spawn_model.launch.py')

    # Declare arguments for the launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty')
    topic = LaunchConfiguration('topic', default='robot_description')
    entity_name = LaunchConfiguration('entity_name', default='diff_drive_bot')
    # Where should the robot be placed
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.1')
    gz_args = LaunchConfiguration('gz_args', default='empty.sdf')

    # Include rsp.launch.py
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include gz_sim.launch.py
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Include gz_spawn_model.launch.py
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_path),
        launch_arguments={
            'world': world,
            'topic': topic,
            'entity_name': entity_name,
            'x': x,
            'y': y,
            'z': z,
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch files to the launch description
    ld.add_action(rsp_launch)
    ld.add_action(gz_sim_launch)
    ld.add_action(spawn_robot)

    return ld
