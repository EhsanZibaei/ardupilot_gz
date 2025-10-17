#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class TFMerger(Node):
    def __init__(self):
        super().__init__('tf_merger')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publisher to /tf
        self.tf_pub = self.create_publisher(TFMessage, '/tf', qos)

        # Subscriptions to /gz/tf and /ap/tf_static
        self.create_subscription(TFMessage, '/gz/tf', self.republish_tf, qos)
        self.create_subscription(TFMessage, '/ap/tf_static', self.republish_tf, qos)
        self.create_subscription(TFMessage, '/tf_robot', self.republish_tf, qos)


        self.get_logger().info("TF Merger running: forwarding /gz/tf + /ap/tf_static -> /tf")

    def republish_tf(self, msg):
        # Direct forward to /tf
        self.tf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
