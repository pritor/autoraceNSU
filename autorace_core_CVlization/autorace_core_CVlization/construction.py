import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.parameter import Parameter

class ConstructionNode(Node):
    def __init__(self):
        super().__init__('construction')

        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cbGoThrough, 1)
        self.pub_state = self.create_publisher(String, '/state', 1)
        self.pub_shtdwn = self.create_publisher(Bool, '/shutdown/lane_follower', 1)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        self.first_stop = False
        self.first_turn = False
        self.second_turn = False
        self.second_stop = False
        self.third_turn = False
        self.fourth_turn = False

        self.timer_node = rclpy.create_node('construction_timer')
        self.timer_node.set_parameters([Parameter('use_sim_time', value=True)])
        self.timer = self.timer_node.create_timer(0.001, self.timerCb)
        self.cur_time = 0



    def timerCb(self):
        # self.get_logger().info('timer_node callback')
        self.cur_time = self.timer_node.get_clock().now().nanoseconds * 1e-9

    def sleep(self, secs):
        rclpy.spin_once(self.timer_node)  # two times, because one is not enough to update cur_time
        rclpy.spin_once(self.timer_node)
        t0 = self.cur_time
        d = 0
        self.get_logger().info(f'timer started with {t0} and {secs}')
        while d <= secs:
            rclpy.spin_once(self.timer_node)
            t = self.cur_time
            d = t - t0
        self.get_logger().info(f'timer ended with {t}')


    def cbGoThrough(self, msg):
        lidar = np.array(msg.ranges)
        min_v = np.min(lidar)
        min_i = np.argmin(lidar)
        if not self.first_stop and lidar[0] < 0.24:
            shtdwn = Bool()
            self.get_logger().info(f'trying to stop {min_i}, {lidar[0]}')
            shtdwn.data = True
            self.first_stop = True
            self.pub_shtdwn.publish(shtdwn)
        if not self.first_turn and lidar[269]>0.19 and self.first_stop:
            self.get_logger().info(f'try to change {lidar[269]}')
            vel = Twist()
            vel.angular.z = 0.5
            self.pub_vel.publish(vel)
        elif not self.first_turn and self.first_stop:
            vel = Twist()
            vel.angular.z = 0.0
            vel.linear.x = 0.15
            self.first_turn = True
            self.pub_vel.publish(vel)
            self.sleep(1.8)
            vel.linear.x = 0.0
            self.pub_vel.publish(vel)
        if self.first_turn and not self.second_turn and not(0.26>lidar[179]>0.12 and 0.7>lidar[0]>0.56):
            vel = Twist()
            vel.angular.z = -0.5
            self.get_logger().info(f'{lidar[179]} at 179, {lidar[0]} at 0')
            self.pub_vel.publish(vel)
        elif self.first_turn and not self.second_turn:
            vel = Twist()
            vel.angular.z = 0.0
            self.get_logger().info(f'stop second_turn {lidar[0]}')
            self.pub_vel.publish(vel)
            self.second_turn = True
        if self.second_turn and lidar[0] > 0.26 and not self.second_stop:
            vel = Twist()
            vel.linear.x = 0.2
            self.get_logger().info(f'{lidar[0]}')
            self.pub_vel.publish(vel)
        elif self.second_turn and not self.second_stop:
            vel = Twist()
            vel.linear.x = 0.0
            self.get_logger().info('stop')
            self.pub_vel.publish(vel)
            self.second_stop = True
        if self.second_stop and lidar[89] > 0.15 and not self.third_turn:
            vel = Twist()
            vel.angular.z = -0.5
            self.get_logger().info(f'{lidar[89]} third turn')
            self.pub_vel.publish(vel)
        elif self.second_stop and not self.third_turn:
            vel = Twist()
            vel.angular.z = 0.0
            vel.linear.x = 0.15
            self.pub_vel.publish(vel)
            self.sleep(2.2)
            vel.linear.x = 0.0
            self.pub_vel.publish(vel)
            self.third_turn = True
        if self.third_turn and not self.fourth_turn and lidar[179] > 0.27:
            vel = Twist()
            vel.angular.z = 0.5
            self.get_logger().info(f'{lidar[179]} fourth turn')
            self.pub_vel.publish(vel)
        elif self.third_turn and not self.fourth_turn and abs(lidar[179] - lidar[180])>0.0005 and lidar[179]<0.27:
            vel = Twist()
            vel.angular.z = 0.3
            self.get_logger().info(f'{lidar[179]} {lidar[180]} fourth turn tuning')
            self.pub_vel.publish(vel)
        elif self.third_turn and not self.fourth_turn:
            vel = Twist()
            vel.angular.z = 0.0
            self.get_logger().info(f'{lidar[179]} stop fourth turn')
            self.pub_vel.publish(vel)
            self.fourth_turn = True
            self.sleep(1.2)
            vel.linear.x = 0.15
            self.pub_vel.publish(vel)
            self.sleep(0.5)
            state = String()
            state.data = 'follow'
            self.turned = True
            self.pub_state.publish(state)
            self.sleep(2)
            state = String()
            state.data = 'completed'
            self.pub_state.publish(state)
            self.destroy_node()

            




def main(args=None):
    rclpy.init(args=args)
    node = ConstructionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
