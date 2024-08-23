""" 00_pysim_ros.py: Provides an example of using ROS to run the main functions of the navigation sim
"""

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import time as pytime
import copy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from assignments.tools.create_sim import NavSim, create_sim


class RosPySim(Node):
    """Create a simulation similar to NavVectorFollower, but the
    main components are run in timer loops defined using ROS2
    """
    def __init__(self, sim: NavSim):
        super().__init__(node_name="RosPySim")
        self.sim = sim

        # Create callback groups
        self.cb_annoying = MutuallyExclusiveCallbackGroup()

        # Create a timer for the annoying slepper (Given as an example)
        self.annoying_timer = self.create_timer(timer_period_sec=5., callback=self.example_timer, callback_group=self.cb_annoying)

    def example_timer(self) -> None:
        """Runs a callback that just delays things to show an example
        """
        self.get_logger().info("Annoying -- sleeping for 3 seconds!!!")
        pytime.sleep(3.)

    def run_sensing(self) -> None:
        """run_sensing is a callback loop to update the sensing within the simulation
        """
        self.get_logger().info("running the sensing loop")
        self.sim.update_sensing()

    def run_dynamics(self) -> None:
        """run_dynamics is a callback loop to update the dynamics of the simulation
        """
        self.get_logger().info("running dynamics loop")

        # Update the current state to be the previous next state (i.e., we're now working with the next state)
        with self.sim.lock:
            self.sim.data.current = copy.deepcopy(self.sim.data.next)

        # Call the sim dynamics update function
        self.sim.update_dynamics()

        # Store the data
        self.sim.store_data_slice(self.sim.data.current)

    def run_plotting(self) -> None:
        """run_plotting is a callback loop to update the plotting
        """
        self.get_logger().info("running plotting timer loop")
        self.sim.update_plot()


def main(args=None):
    # Initialize ros
    rclpy.init(args=args)

    # Create the sim
    sim = create_sim()
    plt.show(block=False)

    # Create and spin node
    node = RosPySim(sim=sim)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
