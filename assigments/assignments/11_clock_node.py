""" Provides a simulated clock with a toggle_pause service """

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration
from rosgraph_msgs.msg import Clock
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty  # Import the Empty service

import threading
import time as pytime


class ClockNode(Node):
	"""Create a simulation clock with a toggle_pause service"""

	def __init__(self):
		super().__init__(node_name="ClockNode")

		self.run_clock: bool = True

		# Declare parameters
		self.declare_parameter("period", 0.01, ParameterDescriptor(description="Time interval between subsequent clock messages"))
		self.declare_parameter("wall_scale", 1.0, ParameterDescriptor(description="Clock speed multiplier for simulation"))

		# Read parameters and store them
		self.period = self.get_parameter("period").value
		if self.period <= 0:
			self.get_logger().warn("Invalid value for 'period'. Setting to default 0.01.")
			self.period = 0.01

		self.wall_scale = self.get_parameter("wall_scale").value
		if self.wall_scale <= 0:
			self.get_logger().warn("Invalid value for 'wall_scale'. Setting to default 1.0.")
			self.wall_scale = 1.0

		self.period_ros = Duration(seconds=self.period)

		# Create a parameter listener
		self.add_on_set_parameters_callback(self.param_callback)

		# Publisher for the clock topic
		self.clock_publisher = self.create_publisher(Clock, "/clock", 10)

		# Create the toggle_pause service
		self.create_service(Empty, "toggle_pause", self.toggle_callback)

	def param_callback(self, params: list[Parameter]) -> SetParametersResult:
		""" Set all of the parameters that are passed in """
		success: bool = True
		for param in params:
			if param.name == "period":
				if param.value > 0:  # Ensure positive values
					self.period = param.value
					self.period_ros = Duration(seconds=self.period)
					self.get_logger().info(f"Updated 'period' to {param.value}")
				else:
					self.get_logger().warn(f"Ignored negative value for 'period': {param.value}")
					success = False
			elif param.name == "wall_scale":
				if param.value > 0:  # Ensure positive values
					self.wall_scale = param.value
					self.get_logger().info(f"Updated 'wall_scale' to {param.value}")
				else:
					self.get_logger().warn(f"Ignored negative value for 'wall_scale': {param.value}")
					success = False
			elif param.name != "use_sim_time":
				msg = f"We don't have a parameter named {param.name}"
				self.get_logger().error(msg)
				success = False

		return SetParametersResult(successful=success)

	def toggle_callback(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
		"""Toggles the run_clock boolean"""
		self.run_clock = not self.run_clock
		state = "running" if self.run_clock else "paused"
		self.get_logger().info(f"Clock is now {state}")
		return res


def main(args=None):
	# Initialize ros
	rclpy.init(args=args)

	# Create the clock node and its executor
	node = ClockNode()
	exec = SingleThreadedExecutor()
	exec.add_node(node)

	# Spin the executor in a different thread
	ros_thread = threading.Thread(target=exec.spin, daemon=True)
	ros_thread.start()

	# Initialize the ros timing variables
	time_latest = Time()
	clock_msg = Clock()
	clock_msg.clock = time_latest.to_msg()
	node.clock_publisher.publish(clock_msg)  # Publish initial clock message

	# Record the initial wall clock time
	wall_time = pytime.perf_counter()

	# Update the plotting
	while rclpy.ok():
		# Check to see if the clock is running
		if not node.run_clock:
			pytime.sleep(1.)
			continue        

		if node.run_clock:  # Only update the clock if run_clock is True
			# Update the time
			time_latest = time_latest + node.period_ros

			# Publish the new clock
			clock_msg.clock = time_latest.to_msg()
			node.clock_publisher.publish(clock_msg)

			# Sleep until the next publication needs to be made
			desired_wall_time = wall_time + node.period / node.wall_scale
			sleep_duration = desired_wall_time - pytime.perf_counter()
			if sleep_duration > 0:
				pytime.sleep(sleep_duration)
				wall_time = desired_wall_time
			else:  # Don't catch up, just go as fast as possible
				wall_time = pytime.perf_counter()

	# Wait for the executor before finishing
	ros_thread.join()


if __name__ == "__main__":
	main()
