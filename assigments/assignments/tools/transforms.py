"""transforms.py Publishes the transforms of the vehicle based on the vehicle pose information. In particular, this node publishes

    map: Static map transform for the world (ENU - East, North, Up)
    vehicle: A frame centered at the vehicle position, but aligned with the world
    body: A frame centered at the vehicle position and aligned with the vehicle: x axis out nose, y axis out left and z pointing up

"""
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import PoseStamped

import numpy as np
import numpy.typing as npt

import tf_transformations
from typing import Any

def rot_z(angle: float) -> npt.NDArray[Any]:
    """Elementary rotation about z-axis

    Args:
        angle: angle of rotation

    Returns:
        rot: rotation matrix about z-axis
    """
    # Calculate angles
    c = np.cos(angle)
    s = np.sin(angle)

    # Calculate rotaiton matrix
    rot = np.array([    [c,   s,  0.],
                        [-s,  c,  0.],
                        [0., 0.,  1.] ])
    return rot

class TransformPublisher(Node):
    """ Given the Vehicle , this class publishes the transforms
        world -> map
        map -> vehicle
        vehicle -> body

    Each transform is prefixed by the given namespace
    """
    def __init__(self) -> None:
        """ Initializes the subscription to the uav state
        """

        # Initialize the node
        super().__init__(node_name="uav_transformer")

        # Create the subsciber for the uav state
        self.sub_pose = self.create_subscription(PoseStamped, "pose", self.state_callback, 1)

        # Create variables for the transforms
        self.br = TransformBroadcaster(self)

        ### Broadcast the transform from map to ned ###
        # Initialize transform
        self._static_br = StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'world'
        static_tf.child_frame_id = 'map'

        # Set zero translation
        static_tf.transform.translation.x = 0.
        static_tf.transform.translation.y = 0.
        static_tf.transform.translation.z = 0.

        # Set rotation
        # rot = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
        # q = tf_transformations.quaternion_from_matrix(to_4x4(rot))
        static_tf.transform.rotation.x = 0.
        static_tf.transform.rotation.y = 0.
        static_tf.transform.rotation.z = 0.
        static_tf.transform.rotation.w = 1.
        self._static_br.sendTransform(static_tf)


    def state_callback(self, msg: PoseStamped) -> None:
        """ Stores the state
        """
        self.publish_transforms(msg)

    def publish_transforms(self, state: PoseStamped) -> None:
        """Publishes the transforms as recorded in the state

        args:
            state: State from which to publish transforms
        """

        # Inertial to vehicle
        stamp = self.get_clock().now().to_msg() # state.pose.header.stamp
        trans = np.array([[state.pose.position.x], [state.pose.position.y], [state.pose.position.z]])
        self.publish_transform(stamp, "map", "vehicle", np.identity(3), trans)

        # Vehicle to body
        q = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w]
        _, _, psi = tf_transformations.euler_from_quaternion(quaternion=q)
        self.publish_transform(stamp, 'vehicle', 'body', rot_z(psi))


    def publish_transform(self,
                          stamp,
                          header_frame: str,
                          child_frame: str,
                          rot: npt.NDArray[Any],
                          trans: npt.NDArray[Any] = np.zeros([3,1])) -> None:
        """Publishes a single transform

        args:
            stamp: Time stamp of transform
            header_frame: Frame from which the transform originates
            child_frame: Frame to which the transform extends
            rot: Rotation matrix defining the rotation from header to child
            trans: Translation vector defining the translation in the header frame to the child frame origin
        """

        # Initialize transform
        t = TransformStamped()
        t.header.stamp = stamp

        # Set frames
        t.header.frame_id = header_frame
        t.child_frame_id = child_frame

        # Set translation
        t.transform.translation.x = trans.item(0)
        t.transform.translation.y = trans.item(1)
        t.transform.translation.z = trans.item(2)

        # Set orientation
        q = tf_transformations.quaternion_from_matrix(to_4x4(rot.T))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.br.sendTransform(t)

def to_4x4(rot: npt.NDArray[Any]) -> npt.NDArray[Any]:
    """ Converts a 3x3 rotation matrix to a 4x4 transformation matrix

    args:
        rot: 3x3 rotation matrix

    returns:
        4x4 rotation matrix
    """
    T = np.zeros([4,4])
    T[0:3,0:3] = rot
    return T

def main(args=None):
    rclpy.init(args=args)

    # Create the transform
    transform_publisher = TransformPublisher()
    rclpy.spin(transform_publisher)

    # # Destroy node instead of waiting for garbage collector
    # transform_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
