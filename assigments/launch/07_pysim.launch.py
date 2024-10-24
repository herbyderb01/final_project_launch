import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config = os.path.join(
      get_package_share_directory('assignments'),
      'rviz',
      '06_pose_scan_world.rviz'
      )
    
    return LaunchDescription([
        Node(
            package='assignments',
            executable='06_pub_sensor',
            name='pub_sensor_06'
        ),
        Node(
            package='assignments',
            executable='06_pub_pose',
            name='pub_pose_06'
        ),
        Node(
            package='assignments',
            executable='transforms',
            name='transform',
        ),
      	Node(
        	package='rviz2',
        	executable='rviz2',
        	name='rviz2',
        	arguments=['-d', rviz_config]
      	)
    ])