import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _clean_frame_id(frame_id: str) -> str:
    return frame_id.lstrip('/')


class OdomToTfNode(Node):
    """Broadcast odometry messages as odom -> base_link TF."""

    def __init__(self) -> None:
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/spot/odometry')
        self.declare_parameter('odom_frame', 'odom_spot')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('prefer_msg_frames', True)

        self._broadcaster = TransformBroadcaster(self)
        topic = str(self.get_parameter('odom_topic').value)
        self.create_subscription(Odometry, topic, self._odom_callback, 20)
        self.get_logger().info(f'Broadcasting TF from odometry topic {topic}')

    def _odom_callback(self, msg: Odometry) -> None:
        prefer_msg_frames = bool(self.get_parameter('prefer_msg_frames').value)
        parent_frame = str(self.get_parameter('odom_frame').value)
        child_frame = str(self.get_parameter('base_frame').value)

        if prefer_msg_frames and msg.header.frame_id:
            parent_frame = msg.header.frame_id
        if prefer_msg_frames and msg.child_frame_id:
            child_frame = msg.child_frame_id

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = _clean_frame_id(parent_frame)
        tf_msg.child_frame_id = _clean_frame_id(child_frame)
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
