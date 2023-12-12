import numpy as np
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class ControlLane(Node):
    def __init__(self):
        super().__init__('control_lane')
        self.sub_lane = self.create_subscription(Float64, '/detect/lane',  self.cbFollowLane, 1)
        # self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size=1)
        self.shtdwn_sub = self.create_subscription(Bool, '/shutdown/lane_follower', self.cbShutdown, 1 )
        self.pub_cmd_vel = self.create_publisher(Twist,  '/cmd_vel',  1)
        # self.pub_ei = self.create_publisher(Float64, '/ei', 1)
        self.lastError = 0.
        self.cumulativeError = 0.
        self.MAX_VEL = 0.15
        self.t0 = self.get_clock().now().nanoseconds * (10**(-9))


        # rospy.on_shutdown(self.fnShutDown)

    # def cbGetMaxVel(self, max_vel_msg):
    #     self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 424

        Kp = 0.01
        Kd = 0.075
        Ki = 0.003
        dt = self.get_clock().now().nanoseconds * (10**(-9)) - self.t0
        e_i = self.cumulativeError/dt
        # ei = Float64()
        # ei.data = e_i
        # self.pub_ei.publish(ei)
        angular_z = Kp * error + Kd * (error - self.lastError) + Ki * e_i
        self.lastError = error
        self.cumulativeError += error

        twist = Twist()
        # twist.linear.x = 0.05
        twist.linear.x = min(self.MAX_VEL * (abs(1 - abs(error) / 424) ** 2.2), 0.8)
        # twist.linear.y = 0
        # twist.linear.z = 0
        # twist.angular.x = 0
        # twist.angular.y = 0
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    def cbShutdown(self, msg):
        if msg.data == True:
            self.get_logger().info('shutdown node, cmd_vel turn to 0')
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            self.destroy_node()
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()