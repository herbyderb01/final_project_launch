import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Paths to packages and configuration files
    diff_drive_bot_pkg_path = get_package_share_directory('diff_drive_bot')
    assignments_pkg_path = get_package_share_directory('assignments')

    xacro_file = os.path.join(diff_drive_bot_pkg_path, 'description', 'robot.urdf.xacro')
    diff_drive_bot_rviz_config = os.path.join(diff_drive_bot_pkg_path, 'config', 'diff_drive_bot.rviz')
    assignments_rviz_config = os.path.join(assignments_pkg_path, 'rviz', 'FP_rviz.rviz')

    # Process the URDF file
    robot_description_config = xacro.process_file(xacro_file)

    # Use simulation time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (sim) time if true'
    )

    # Nodes from the first launch file
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # rviz_node_diff_drive = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2_diff_drive',
    #     arguments=['-d', diff_drive_bot_rviz_config],
    #     parameters=[{'use_sim_time': True}]
    # )

    # Nodes from the second launch file
    pub_sensor_node = Node(
        package='assignments',
        executable='06_pub_sensor',
        name='pub_sensor_06'
    )

    pub_pose_node = Node(
        package='assignments',
        executable='09_pub_pose',
        name='pub_pose_09'
    )

    planning_service_node = Node(
        package='assignments',
        executable='09_planning_service',
        name='planner'
    )

    transform_node = Node(
        package='assignments',
        executable='transforms',
        name='transform',
    )

    rviz_node_assignments = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_assignments',
        arguments=['-d', assignments_rviz_config]
    )

    # Return LaunchDescription combining both setups
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # rviz_node_diff_drive,
        pub_sensor_node,
        pub_pose_node,
        planning_service_node,
        transform_node,
        rviz_node_assignments
    ])
