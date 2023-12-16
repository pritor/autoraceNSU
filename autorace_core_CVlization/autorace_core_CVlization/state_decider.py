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
            self.get_logger().info('lane following subprocess run')
            subprocess.Popen(["ros2", "run", "autorace_core_CVlization", "lane_follower"])

        elif state == "intersection":
            if not self.mission_running:
                self.get_logger().info('intersection subprocess run')
                self.mission_running = True
                subprocess.Popen(["ros2", "run", "autorace_core_CVlization", "intersection"])

        elif state == "construction":
            if not self.mission_running:
                self.get_logger().info('construction subprocess run')
                self.mission_running = True
                subprocess.Popen(['ros2', 'run', 'autorace_core_CVlization', 'construction'])

        elif state == "parking":
            if not self.mission_running:
                self.get_logger().info('parking subprocess run')
                self.mission_running = True
                subprocess.Popen(["ros2", "run", "autorace_core_CVlization", "parking"])

        elif state == "pedestrian":
            self.get_logger().info('pedestrian subprocess run')
            self.mission_running = True

        elif state == "tunnel":
            self.get_logger().info('tunnel subprocess run')
            self.mission_running = True

        elif state == "completed":
            self.get_logger().info('ready for new missions')
            self.mission_running = False


def main(args=None):
    rclpy.init(args=args)
    node = StateDecider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()