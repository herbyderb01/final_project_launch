import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult

import numpy as np

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(node_name="Hello_world")
    node.get_logger().info("hello world")

    rclpy.spin(node)


if __name__ == '__main__':
    main()
