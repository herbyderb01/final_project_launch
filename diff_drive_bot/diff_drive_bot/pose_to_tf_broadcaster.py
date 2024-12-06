import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PoseToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_to_tf_broadcaster')
        self.subscription = self.create_subscription(
            Pose,
            '/pose',
            self.pose_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Node initialized and subscribing to /pose.")

    def pose_callback(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'  # Parent frame
        t.child_frame_id = 'base_link'  # Robot frame

        # Populate the transform from the Pose message
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z
        t.transform.rotation = msg.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
