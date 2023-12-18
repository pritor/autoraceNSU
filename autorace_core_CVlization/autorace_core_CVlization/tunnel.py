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
        self.first_turn = False
        self.second_turn = False
        self.obstacle = False

        self.lastError = 0.0



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
        min_i = np.argmin(lidar)
        if not self.started:
            # if lidar[270] < 0.23 and lidar[90] > 1.0:
            stop = Bool()
            stop.data = True
            self.pub_shtdwn.publish(stop)
            self.sleep(2.0)
            self.started = True
            vel = Twist()
            vel.linear.x = 0.0
            self.pub_vel.publish(vel)
            self.get_logger().info(f'{lidar[90]}, {lidar[270]}')
            self.sleep(1.5)
            vel.linear.x = 0.2
            vel.angular.z = 0.0
            self.pub_vel.publish(vel)
            self.sleep(2.0)
            self.get_logger().info('stop turning')
            vel.linear.x = 0.0
            vel.angular.z = 0.5
            self.pub_vel.publish(vel)
            self.sleep(1.5)
            vel.linear.x = 0.2
            vel.angular.z = 0.0
            self.pub_vel.publish(vel)
            self.sleep(2.0)
        elif self.started and not self.obstacle and lidar[0] > 0.2:
            if lidar[90] > 1.0: lidar[90] = 1.0
            error = lidar[90] - 0.16
            self.get_logger().info(f'error = {error}')
            vel = Twist()
            vel.linear.x = min(0.2 * (abs(1 - abs(error) / 424) ** 2.2), 0.8)
            vel.angular.z = 3.0 * error + 5.0 * (error - self.lastError)
            self.lastError = error
            self.pub_vel.publish(vel)
        elif self.started and not self.obstacle and not self.first_turn:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = -0.2
            self.pub_vel.publish(vel)
            self.get_logger().info('turning')
            if lidar[180] < 0.2:
                self.first_turn = True
                self.get_logger().info('turned')

        # elif self.started and not self.first_turn and lidar[180] > 0.10:
        #     vel = Twist()
        #     vel.linear.x = 0.02
        #     vel.angular.z = 0.3
        #     self.pub_vel.publish(vel)
        #     self.get_logger().info(f'first turn tuning {min_i} to 180')
        # elif self.started and not self.first_turn:
        #     vel = Twist()
        #     vel.linear.x = 0.0
        #     vel.angular.z = 0.0
        #     self.pub_vel.publish(vel)
        #     self.get_logger().info(f'tuned first_turn {min_i}')
        #     self.first_turn = True
        #     self.sleep(1.0)
        #     vel.linear.x = 0.2
        #     self.pub_vel.publish(vel)
        #     self.sleep(1.0)
        #     vel.linear.x = 0.0
        #     self.pub_vel.publish(vel)





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
