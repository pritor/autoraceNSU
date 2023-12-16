import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import subprocess
from std_msgs.msg import Bool


class ped_pepe(Node):

    def __init__(self):
        super().__init__('pedestrian_pepe')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #ros2 topic pub --once /shutdown/lane_follower std_msgs/Bool 'data: True'
        self.stop_lane_follower_publisher = self.create_publisher(Bool, '/shutdown/lane_follower', 10)
        self.run_lane_follower_publisher = self.create_publisher(String, '/state', 10)
        self.subscriber_1 = self.create_subscription(LaserScan, '/scan', self.log_func, 10)
        self.subscriber_1 = self.create_subscription(LaserScan, '/scan', self.watch_pedestrian, 10)
        self.scan = LaserScan()
        #self.timer = self.create_timer(0.1, self.move)
        #self.timer_back = self.create_timer(0.1, self.back)
        self.flag_pepe = False
        self.get_logger().info('pedestrian_node initialized')

    def log_func(self, msg):
        laser = np.array(msg.ranges)

        if len(laser) != 0:
            min_val = min(laser)
            index_min_val = np.argmin(laser)
            self.get_logger().info(f' 	from log index {index_min_val}, value {min_val}')

    def watch_pedestrian(self, msg):
    	#stop command
    	laser = np.array(msg.ranges)
    	min_val = min(laser)
    	index_min_val = np.argmin(laser)

    	if (((index_min_val>=0 and index_min_val <= 28) or (index_min_val>=355 and index_min_val <= 359)) and (min_val <= 0.15)):
    		self.get_logger().info(f'STOP, index {index_min_val}, val {min_val}')
    		stop_msg = Bool()
    		stop_msg.data = True
    		self.stop_lane_follower_publisher.publish(stop_msg)
    		self.flag_pepe = True
    	elif (index_min_val>=110 and index_min_val <= 235):
    		self.get_logger().info('pedestrian mission done')
    		run_msg = String()
    		run_msg.data = 'follow'
    		self.run_lane_follower_publisher.publish(run_msg)
    		self.destroy_node()
    	else:
    		#prodolzhaem dvizhenie
    		if (self.flag_pepe):
    			run_msg = String()
    			run_msg.data = 'follow'
    			self.run_lane_follower_publisher.publish(run_msg)
    			self.flag_pepe = True
    		self.get_logger().info('running')




def main(args=None):
    rclpy.init(args=args)
    node = ped_pepe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()