import rclpy
from rclpy.node import Node

from nav_msgs.msg._path import Path
from nav_msgs.srv._get_plan import GetPlan
from geometry_msgs.msg._pose_stamped import PoseStamped

from py_sim.path_planning.path_generation import create_path
from py_sim.worlds.polygon_world import (
	generate_non_convex_obstacles,
)
from py_sim.tools.sim_types import TwoDimArray


class PlannerNode(Node):
	"""Creates a minimal planning node
	"""
	def __init__(self):
		super().__init__(node_name="RosSensorSim")

		# Create the obstacle environment over which the planning occurs
		self.obstacle_world = generate_non_convex_obstacles()

		# Advertise the service 
		self.srv = self.create_service(GetPlan, 'get_plan', self.create_plan)

		# Add a publisher for the path (purely for visualization)
		self.pub = self.create_publisher(Path, "planned_path", 10)


	def create_plan(self, req: GetPlan.Request, res: GetPlan.Response) -> GetPlan.Response:
		"""Creates a plan given the request

		Args:
			req: The requested start and end points of the plan
			res: The resulting path (also returned)

		Returns:
			res: The resulting path
		"""
		# Check the header information - not performing transform
		if req.start.header.frame_id != "map" or req.goal.header.frame_id != "map":
			self.get_logger().error("A frame other than map was received. No transforms implemented yet, so not planning")
			res.plan.header = req.start.header
			return res

		# Create a plan
		plan = create_path(start=TwoDimArray(x=req.start.pose.position.x, y = req.start.pose.position.y),
						   end=TwoDimArray(x=req.goal.pose.position.x, y = req.goal.pose.position.y),
						   obstacle_world=self.obstacle_world,
						   plan_type="voronoi")

		# Check the resulting plan
		if plan is None:
			self.get_logger().error("Unable to find a valid plan")
			res.plan.header = req.start.header
			return res

		# Convert the plan to a navigation path
		for (x,y) in zip(plan[0], plan[1]):
			pose = PoseStamped()
			pose.header.frame_id = "map"
			pose.header.stamp = self.get_clock().now().to_msg()
			pose.pose.position.x = x
			pose.pose.position.y = y
			pose.pose.position.z = 0.
			res.plan.poses.append(pose)
		res.plan.header.frame_id = "map"
		res.plan.header.stamp = self.get_clock().now().to_msg()

		# Publish the planned path
		self.pub.publish(res.plan)
		return res

def main(args=None):
	# Initialize ROS 2
	rclpy.init(args=args)

	# Create the PlannerNode instance
	planner_node = PlannerNode()

	# Spin to keep the node active and respond to service requests
	rclpy.spin(planner_node)

	# Clean up after shutdown
	planner_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()