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


class RosPySim(Node):
    """Create a simulation similar to NavVectorFollower, but the
    main components are run in timer loops defined using ROS2
    """
    def __init__(self, sim: NavSim):
        super().__init__(node_name="RosPySim")
        self.sim = sim

        # Create publishers
        self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)
        self.pub_scan = self.create_publisher(LaserScan, 'scan', 10)

        # Create callback groups
        self.cb_annoying = MutuallyExclusiveCallbackGroup()
        self.cb_sensing = MutuallyExclusiveCallbackGroup()
        self.cb_dynamics = MutuallyExclusiveCallbackGroup()
        self.cb_plotting = MutuallyExclusiveCallbackGroup()

        # Create a timer for the annoying slepper (Given as an example)
        self.annoying_timer = self.create_timer(timer_period_sec=5., callback=self.example_timer, callback_group=self.cb_annoying)
        self.sensing_timer = self.create_timer(timer_period_sec=sim.params.sim_step, callback=self.run_sensing, callback_group=self.cb_sensing)
        self.dynamics_timer = self.create_timer(timer_period_sec=sim.params.sim_step, callback=self.run_dynamics, callback_group=self.cb_dynamics)
        # self.plotting_timer = self.create_timer(timer_period_sec=sim.params.sim_plot_period, callback=self.run_plotting, callback_group=self.cb_plotting)

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

    def example_timer(self) -> None:
        """Runs a callback that just delays things to show an example
        """
        # self.get_logger().info("Annoying -- sleeping for 3 seconds!!!")
        pytime.sleep(3.)

    def run_sensing(self) -> None:
        """run_sensing is a callback loop to update the sensing within the simulation
        """
        # self.get_logger().info("running the sensing loop")
        self.sim.update_sensing()

        # Publish scan data
        self.publish_scan(sensor=self.sim.sensor, meas=self.sim.data.range_bearing_latest)

    def run_dynamics(self) -> None:
        """run_dynamics is a callback loop to update the dynamics of the simulation
        """
        # self.get_logger().info("running dynamics loop")

        # Update the current state to be the previous next state (i.e., we're now working with the next state)
        with self.sim.lock:
            self.sim.data.current = copy.deepcopy(self.sim.data.next)

        # Call the sim dynamics update function
        self.sim.update_dynamics()

        # Store the data
        self.sim.store_data_slice(self.sim.data.current)

        # Run pose publisher
        self.publish_pose()

    def run_plotting(self) -> None:
        """run_plotting is a callback loop to update the plotting
        """
        # self.get_logger().info("running plotting timer loop")
        self.sim.update_plot()


def main(args=None):
	# Initialize ros
	rclpy.init(args=args)

	# Create the sim
	sim = create_sim()
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
    # 
