""" 00_pysim_ros.py: Provides an example of using ROS to run the main functions of the navigation sim
"""
import threading
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import time as pytime
import copy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from assignments.tools.create_sim import NavSim, create_sim
from tf_transformations import quaternion_from_euler

from py_sim.tools.sim_types import RangeBearingMeasurements
from py_sim.sensors.range_bearing import RangeBearingSensor

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from typing import Optional
import tf_transformations
import geometry_msgs


class RosSensorSim(Node):
    """Create a simulation similar to NavVectorFollower, but the
    main components are run in timer loops defined using ROS2
    """
    def __init__(self, sim: NavSim):
        super().__init__(node_name="RosSensorSim")
        self.sim = sim

        # Create publishers
        # self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)
        self.pub_scan = self.create_publisher(LaserScan, 'scan', 10)
        self.pub_world = self.create_publisher(Marker, 'world_visualization', 10)
        
        self.pose_latest: Optional[PoseStamped] = None # pose message storage variable
        

        # Create callback groups
        self.cb_sensing = MutuallyExclusiveCallbackGroup()
        self.cb_plotting = MutuallyExclusiveCallbackGroup()
        self.cb_world = MutuallyExclusiveCallbackGroup()

        # Create subscribers
        self.sub_pose = self.create_subscription(PoseStamped, "pose", self.pose_callback, 10, callback_group=self.cb_sensing)

        # Create a timer
        self.sensing_timer = self.create_timer(timer_period_sec=sim.params.sim_step, callback=self.run_sensing, callback_group=self.cb_sensing)
        self.sensing_timer = self.create_timer(timer_period_sec=1, callback=self.publish_world, callback_group=self.cb_world)

    def publish_scan(self, sensor: RangeBearingSensor, meas: RangeBearingMeasurements) -> None:
        """Publishes the current range-bearing scan

        Args:
            sensor: The sensor being used for the range bearing measurements
            meas: The measurement received from the sensor
        """

        # Create the message
        scan = LaserScan()
        scan.header.frame_id = "body"
        scan.header.stamp = self.get_clock().now().to_msg()

        # Populate the laser scan message
        scan.angle_min = sensor.orien[0]
        scan.angle_max = sensor.orien[-1]
        scan.angle_increment = sensor.delta
        scan.time_increment = 0.
        scan.scan_time = 0.
        scan.range_min = 0.
        scan.range_max = sensor.max_dist
        scan.ranges = copy.deepcopy(meas.range)

        # Publish the scan
        self.pub_scan.publish(scan)

    def run_sensing(self) -> None:
        """run_sensing is a callback loop to update the sensing within the simulation
        """
        # self.get_logger().info("running the sensing loop")
        self.sim.update_sensing()

        # Publish scan data
        self.publish_scan(sensor=self.sim.sensor, meas=self.sim.data.range_bearing_latest)
        
        # Only process the pose if data has been received
        if self.pose_latest is None:
            return

        # Extract the orientation from the quaternion
        q = [self.pose_latest.pose.orientation.x, self.pose_latest.pose.orientation.y, self.pose_latest.pose.orientation.z, self.pose_latest.pose.orientation.w]
        _, _, psi = tf_transformations.euler_from_quaternion(quaternion=q)
        self.sim.data.current.state.psi = psi

        # Update the position
        self.sim.data.current.state.x = self.pose_latest.pose.position.x
        self.sim.data.current.state.y = self.pose_latest.pose.position.y

    def pose_callback(self, msg: PoseStamped) -> None:
        """ Stores the latest pose stamped message
        """
        self.pose_latest = msg

    def publish_world(self) -> None:
        """Publish the world messages
        """
        # Create the message to publish
        lines = Marker()
        lines.header.frame_id = "map"
        lines.header.stamp = self.get_clock().now().to_msg()
        lines.ns = "world_vis"
        lines.id = 100
        lines.type = Marker.LINE_LIST
        lines.action = Marker.MODIFY
        lines.scale.x = 0.1 # Scaling (size) of the line
        lines.scale.y = 0.1
        lines.scale.z = 1.
        lines.color.r = 1. # Color of the line
        lines.color.g = 0.
        lines.color.b = 0.
        lines.color.a = 1.
        lines.frame_locked = True

        # Add the points of the world onto the line
        for edge in self.sim.world.edges:
            # Add point corresponding to the first column of edge
            pnt1 = Point()
            pnt1.x = edge[0,0]
            pnt1.y = edge[1,0]
            pnt1.z = 0.
            lines.points.append(pnt1)

            # Add point corresponding to the second column of edge
            pnt2 = Point()
            pnt2.x = edge[0,1]
            pnt2.y = edge[1,1]
            pnt2.z = 0.
            lines.points.append(pnt2)

        # Publish the points
        self.pub_world.publish(lines)
        
def main(args=None):
	# Initialize ros
	rclpy.init(args=args)

	# Create the sim
	sim = create_sim()
	plt.show(block=False)

	# Create and spin node
	node = RosSensorSim(sim=sim)
	# rclpy.spin(node)
	exec = MultiThreadedExecutor()
	exec.add_node(node)
	# exec.spin()
	ros_thread = threading.Thread(target=exec.spin, daemon=True)
	ros_thread.start()
        
	# node.ros_thread.join()
    
    # Keep the script running, you can add additional logic here if needed
	try:
	 while rclpy.ok():
			pytime.sleep(0.1)  # Avoids high CPU usage
	except KeyboardInterrupt:
		pass
	
	# Shutdown and clean up
	exec.shutdown()
	ros_thread.join()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
