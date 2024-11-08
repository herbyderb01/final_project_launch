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
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

from sensor_msgs.msg import LaserScan
from py_sim.tools.sim_types import RangeBearingMeasurements
from py_sim.sensors.range_bearing import RangeBearingSensor

from typing import Optional
import tf_transformations

from nav_msgs.srv._get_plan import GetPlan
from geometry_msgs.msg._pose_stamped import PoseStamped
from typing import cast
import numpy as np
from py_sim.tools.projections import LineCarrot
import typing

class RosPySim(Node):
	"""Create a simulation similar to NavVectorFollower, but the
	main components are run in timer loops defined using ROS2
	"""
	def __init__(self, sim: NavSim):
		super().__init__(node_name="RosPySim")
		self.sim = sim
		
		self.run_robot = False
		self.future = None

		# Create the client
		self.plan_client = self.create_client(GetPlan, "get_plan")

		# Create publishers
		self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)

		
		# Create callback groups
		self.cb_plan = MutuallyExclusiveCallbackGroup()
		self.cb_dynamics = MutuallyExclusiveCallbackGroup()
		self.cb_plotting = MutuallyExclusiveCallbackGroup()

		# Create subscribers
		self.goal_pose = self.create_subscription(PoseStamped, "goal_pose", self.goal_callback, 10, callback_group=self.cb_plan)
		self.sub_pose = self.create_subscription(LaserScan, "scan", self.scan_callback, 10, callback_group=self.cb_dynamics)
		self.sense: Optional[PoseStamped] = None # pose message storage variable

		# Create timers
		self.dynamics_timer = self.create_timer(timer_period_sec=sim.params.sim_step, callback=self.run_dynamics, callback_group=self.cb_dynamics)

	def result_callback(self, future: rclpy.Future) -> None:
		"""Calculates the resulting path
		"""
		# Extract the result from the future
		result = cast(GetPlan.Response, future.result())

		# Extract the plan's (x,y) values
		x_vec: list[float] = []
		y_vec: list[float] = []
		for pose in result.plan.poses:
			x_vec.append(pose.pose.position.x)
			y_vec.append(pose.pose.position.y)

		# Store the path and update the carrot follower
		line = np.array([x_vec, y_vec])
		self.sim.carrot = LineCarrot(line=line, s_dev_max=5., s_carrot=2.)

		# Indicate that a path has been received
		self.run_robot = True

	def goal_callback(self, msg: PoseStamped) -> None:
		"""Creates a service call to make a plan given the current vehicle
			position and the goal position defined by the msg

		Args:
			msg: Defines the goal position for the plan
		"""
		# Create the get_plan message
		srv_req = GetPlan.Request()

		# Create the starting location based off of the current position
		srv_req.start.header.frame_id = "map"
		srv_req.start.header.stamp = self.get_clock().now().to_msg()
		srv_req.start.pose.position.x = self.sim.data.current.state.x
		srv_req.start.pose.position.y = self.sim.data.current.state.y

		# Create the goal location based off of the callback message
		srv_req.goal = copy.deepcopy(msg)

		# Call the service
		self.future = self.plan_client.call_async(srv_req)
		self.future.add_done_callback(self.result_callback)

	
	def publish_pose(self) -> None:
		"""Publishes the current vehicle position and orientation
		"""
		# Create the pose
		pose = PoseStamped()
		pose.header.frame_id = "map"
		pose.header.stamp = self.get_clock().now().to_msg()

		# Populate the pose position
		pose.pose.position.x = self.sim.data.current.state.x
		pose.pose.position.y = self.sim.data.current.state.y
		pose.pose.position.z = 0.

		# Populate the pose orientation
		quat = quaternion_from_euler(0., 0., self.sim.data.current.state.psi)
		pose.pose.orientation.x = quat[0]
		pose.pose.orientation.y = quat[1]
		pose.pose.orientation.z = quat[2]
		pose.pose.orientation.w = quat[3]

		# Publish the message
		self.pub_pose.publish(pose)
		
	def run_dynamics(self) -> None:
		"""run_dynamics is a callback loop to update the dynamics of the simulation
		"""
		# self.get_logger().info("running dynamics loop")

		# Update the current state to be the previous next state (i.e., we're now working with the next state)
		with self.sim.lock:
			self.sim.data.current = copy.deepcopy(self.sim.data.next)

		if self.run_robot:
			# Call the sim dynamics update function
			self.sim.update_dynamics()
		else:
			self.sim.data.next.time = self.sim.data.current.time + self.sim.params.sim_step

		# Store the data
		self.sim.store_data_slice(self.sim.data.current)

		# Run pose publisher
		self.publish_pose()
		
		# Update the sensor measurements
		if self.sense is not None:
			self.sim.data.range_bearing_latest = self.sim.sensor.create_measurement_from_range(
				pose=self.sim.data.current.state,
				ranges=self.sense.ranges
			)

	def run_plotting(self) -> None:
		"""run_plotting is a callback loop to update the plotting
		"""
		# self.get_logger().info("running plotting timer loop")
		self.sim.update_plot()

	def scan_callback(self, msg: PoseStamped) -> None:
		""" Stores the latest pose stamped message
		"""
		self.pose_latest = msg
		
def main(args=None):
	# Initialize ros
	rclpy.init(args=args)

	# Create the sim
	sim = create_sim(create_initial_plan=False)
	plt.show(block=False)

	# Create and spin node
	node = RosPySim(sim=sim)
	# rclpy.spin(node)
	exec = MultiThreadedExecutor()
	exec.add_node(node)
	# exec.spin()
	ros_thread = threading.Thread(target=exec.spin, daemon=True)
	ros_thread.start()
	
	rate = node.create_rate(frequency=1/sim.params.sim_plot_period)
	while rclpy.ok():
		node.sim.update_plot()
		rate.sleep()
	
	node.ros_thread.join()


if __name__ == '__main__':
	main()
