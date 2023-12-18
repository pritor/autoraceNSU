import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import numpy as np
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
import time


class TunnelNode(Node):
    def __init__(self):
        super().__init__('tunnel')
        self.sub_depth = self.create_subscription(Image, '/depth/image', self.cbDepth, 1)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cbLidar, 1)
        self.pub_state = self.create_publisher(String, '/state', 1)
        self.pub_shtdwn = self.create_publisher(Bool, '/shutdown/lane_follower', 1)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_debug = self.create_publisher(Image, '/depth/debug', 1)

        self.timer_node = rclpy.create_node('tunnel_timer')
        self.timer_node.set_parameters([Parameter('use_sim_time', value=True)])
        self.timer = self.timer_node.create_timer(0.0001, self.timerCb)
        self.cur_time = 0

        self.started = False
        self.turn = False
        self.is_dealing_with_obstacle = False

        stop = Bool()
        stop.data = True
        self.pub_shtdwn.publish(stop)
        self.sleep(0.4)
        vel = Twist()
        vel.linear.x = 0.2
        self.pub_vel.publish(vel)


    def timerCb(self):
        # self.get_logger().info('timer_node callback')
        self.cur_time = self.timer_node.get_clock().now().nanoseconds * 1e-9

    def sleep(self, secs):
        rclpy.spin_once(self.timer_node)  # two times, because one is not enough to update cur_time
        rclpy.spin_once(self.timer_node)
        time.sleep(0.001)
        rclpy.spin_once(self.timer_node)
        t0 = self.cur_time
        d = 0
        self.get_logger().info(f'timer started with {t0} and {secs}')
        while d <= secs:
            rclpy.spin_once(self.timer_node)
            t = self.cur_time
            d = t - t0
        self.get_logger().info(f'timer ended with {t}')

    def cbLidar(self, msg):
        lidar = np.array(msg.ranges)
        if not self.started:
            if lidar[270] < 100:
                self.started = True
                vel = Twist()
                vel.linear.x = 0.0
                self.pub_vel.publish(vel)
                self.get_logger().info(f'{lidar[90]}, {lidar[270]}')
        else:

            self.get_logger().info('started')

    def cbDepth(self, msg):
        pass




def main(args=None):
    rclpy.init(args=args)
    node = TunnelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
