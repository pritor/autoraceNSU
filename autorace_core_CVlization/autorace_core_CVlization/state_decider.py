import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import subprocess


class StateDecider(Node):
    def __init__(self):
        super().__init__('state_decider')
        self.state_sub = self.create_subscription(String, '/state', self.decider_callback, 1)
        self.mission_running = False

    def decider_callback(self, msg):
        state = msg.data
        if state == "follow":
            self.get_logger().info('starting subprocess run')
            self.mission_running = False
            subprocess.call(["ros2", "run", "autorace_core_CVlization", "lane_follower"])

        elif state == "intersection":
            self.get_logger().info('intersection subprocess run')
            self.mssion_running = True

        elif state == "construction":
            self.get_logger().info('construction subprocess run')
            self.mssion_running = True

        elif state == "parking":
            self.get_logger().info('parking subprocess run')
            self.mssion_running = True

        elif state == "pedestrian":
            self.get_logger().info('pedestrian subprocess run')
            self.mssion_running = True

        elif state == "tunnel":
            self.get_logger().info('parking subprocess run')
            self.mssion_running = True


def main(args=None):
    rclpy.init(args=args)
    node = StateDecider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()