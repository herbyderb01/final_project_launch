from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	rviz_config = os.path.join(
		get_package_share_directory('assignments'),
		'rviz',
		'11_sim_clock.rviz'
	)
	
	return LaunchDescription([
		Node(
			package='assignments',
			executable='06_pub_sensor',
			name='pub_sensor_06',
			parameters=[{
				'use_sim_time': True
			}]
		),
		Node(
			package='assignments',
			executable='09_pub_pose',
			name='pub_pose_09',
			parameters=[{
				'use_sim_time': True
			}]
		),
		Node(
			package='assignments',
			executable='09_planning_service',
			name='planner',
			parameters=[{
				'use_sim_time': True
			}]
		),
		Node(
			package='assignments',
			executable='transforms',
			name='transform',
			parameters=[{
				'use_sim_time': True
			}]
		),
	  	Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			arguments=['-d', rviz_config],
			parameters=[{
				'use_sim_time': True
			}]
		),
	  	Node(
			package='rqt_gui',
			executable='rqt_gui',
			name='rqt',
			parameters=[{
				'use_sim_time': False
			}]
		),
	  	Node(
			package='assignments',
			executable='11_clock',
			name='clock',
			parameters=[{
				'use_sim_time': False
			}]
		)
	])