from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
import xacro


def generate_launch_description():
 
	 # Check if we're told to use sim time
	use_sim_time = LaunchConfiguration('use_sim_time')

	# Process the URDF file
	pkg_path = os.path.join(get_package_share_directory('diff_drive_bot'))
	xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
	rviz_config = os.path.join(pkg_path,'config','diff_drive_bot.rviz')
	robot_description_config = xacro.process_file(xacro_file)
	
	# Create a robot_state_publisher node
	params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
	
	node_robot_state_publisher = Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			output='screen',
			parameters=[params]
	)
 
	# # tf bot positions
	# pos_tf_pub = Node(
    #         package='diff_drive_bot',
    #         executable='pose_to_tf_broadcaster',
    #         name='pose_to_tf_broadcaster'
    #     )
 
	# Create rviz2 node
	rviz_node = Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
   			arguments=['-d', rviz_config],
			parameters=[{
				'use_sim_time': True
			}]
		)

	# Create joint_state_publisher_gui node for wheels
	joint_state_publisher_gui_node = Node(
			package='joint_state_publisher_gui',
			executable='joint_state_publisher_gui',
			name='joint_state_publisher_gui'
		)
	
	return LaunchDescription([
	  	DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='Use sim time if true'),
    
		# pos_tf_pub,
    
    	rviz_node,
     
    	node_robot_state_publisher,
     
     	joint_state_publisher_gui_node
	])