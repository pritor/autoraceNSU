import numpy as np
from geometry_msgs.msg import Twist
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


class Laser_pepe(Node):

    def __init__(self):
        super().__init__('laser_pepe')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #ros2 topic pub --once /shutdown/lane_follower std_msgs/Bool 'data: True'
        self.stop_lane_follower_publisher = self.create_publisher(Bool, '/shutdown/lane_follower', 10)
        self.subscriber_1 = self.create_subscription(LaserScan, '/scan', self.log_func, 10)
        self.subscriber_2 = self.create_subscription(LaserScan, '/scan', self.start_and_stop_1, 10)
        self.subscriber_2 = self.create_subscription(LaserScan, '/scan', self.find_place, 10)
        self.subscriber_3 = self.create_subscription(LaserScan, '/scan', self.move, 10)
        self.scan = LaserScan()
        #self.timer = self.create_timer(0.1, self.move)
        self.timer_back = self.create_timer(0.1, self.back)
        self.timer_end = self.create_timer(0.1, self.end_of_mission)
        self.begin = True
        self.go_turn_1 = False
        self.go_turn_2 = False
        self.park = False
        self.go_back = False
        self.turned_right = False
        self.is_mission_complete = False
        self.control_index = 0
        self.flag_pepe = True
        self.get_logger().info('laser_pepe initialized')

    def log_func(self, msg):
        laser = np.array(msg.ranges)

        if len(laser) != 0:
            min_val = min(laser)
            index_min_val = np.argmin(laser)
            self.get_logger().info(f'near_values {index_min_val}')
        
    def start_and_stop_1(self, msg):
        flag = True
        if (self.begin):
            #robot is moving on line_folower
            #ranges = [218:235+5?]
            self.get_logger().info('STARTED MISSION')
            value_max = 0
            value_min = 0
            laser = np.array(msg.ranges)
            min_val = min(laser)
            index_min_val = np.argmin(laser)

            self.get_logger().info('min index is ' + str(index_min_val))
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            if (index_min_val>210 and index_min_val<230):
                self.control_index = index_min_val
                flag = False
                stop_msg = Bool()        
                stop_msg.data = True
                self.stop_lane_follower_publisher.publish(stop_msg)
                self.bedin = False
                self.go_turn_1 = True




    def move(self, msg):
        if (self.go_turn_1):

            self.get_logger().info('TURNING LEFT')

            PI = 3.1415926535897
            #stop line_finder

            #turn on 90o
            laser = np.array(msg.ranges)
            min_val = min(laser)
            index_min_val = np.argmin(laser)

            if (self.flag_pepe ):
                self.control_index = index_min_val
                self.flag_pepe = False

            angle = 60
            speed = 0.4
            angular_speed = speed*2*PI/360
            relative_angle = (angle*2*PI/360)/10
            vel_msg = Twist()
            vel_msg.angular.z = abs(angular_speed*100)
            #ros1 = Time.now().to_sec(), ROS2 - ????
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            current_angle = 0
            #self.control_index + 90
            #131 = 125
            if (index_min_val > (128)):
                self.get_logger().info(f'TURNING PEPE {index_min_val } {125}')
                self.publisher.publish(vel_msg)
                t1 = self.get_clock().now().nanoseconds * (10**(-9))
                current_angle = angular_speed*((t1-t0))
            else:
                self.get_logger().info(f'MOVING PEPE {index_min_val } {125}')
                vel_msg.angular.z = 0.0
                self.publisher.publish(vel_msg)

                #during some time: vel_msg.linear.x = 0.4
                t0 = self.get_clock().now().nanoseconds * (10**(-9))
                t1 = self.get_clock().now().nanoseconds * (10**(-9))
                moving_seconds = 19
                vel_msg.linear.x = 0.4
                while((t1-t0)<moving_seconds):
                    t1 = self.get_clock().now().nanoseconds * (10**(-9))
                    self.get_logger().info(f'GOING PEPE {t1-t0}')
                    self.publisher.publish(vel_msg)

                vel_msg.linear.x = 0.0
                self.publisher.publish(vel_msg)

                #go to find_place
                self.go_turn_1 = False
                self.go_turn_2 = True
            
        

    def find_place(self, msg):
        if(self.go_turn_2):
            PI = 3.1415926535897
            self.get_logger().info('FINDING FREE PLACE')
            laser = np.array(msg.ranges)
            min_values = [0.11912059038877487, 0.12555094063282013]
            ranges_left = laser[73:99]
            ranges_right = laser[255:288]
            min_val = min(laser)
            index_min_val = np.argmin(laser)
            angle = 90
            speed = 0.4
            angular_speed = speed*2*PI/360
            relative_angle = angle*2*PI/360
            vel_msg = Twist()
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #if there are some object on robot's left/right, turn right/left --- if_construction
            if (index_min_val>73 and index_min_val<99):
                #car on the left side, turn right
                self.turned_right = True
                vel_msg.angular.z = abs(angular_speed)
                #ros1 = Time.now().to_sec(), ROS2 - ????
                t0 = self.get_clock().now().nanoseconds * (10**(-9))
                current_angle = 0
                while(current_angle < relative_angle):
                    self.publisher.publish(vel_msg)
                    t1 = self.get_clock().now().nanoseconds * (10**(-9))
                    current_angle = angular_speed*(t1-t0)

                vel_msg.angular.z = 0
                self.publisher.publish(vel_msg)
                
            elif (index_min_val>255 and index_min_val<288):
                #car on the right side, turn left
                vel_msg.angular.z = -abs(angular_speed)
                #ros1 = Time.now().to_sec(), ROS2 - ????
                t0 = self.get_clock().now().nanoseconds * (10**(-9))
                current_angle = 0
                while(current_angle < relative_angle):
                    self.publisher.publish(vel_msg)
                    t1 = self.get_clock().now().nanoseconds * (10**(-9))
                    current_angle = angular_speed*(t1-t0)

                vel_msg.angular.z = 0
                self.publisher.publish(vel_msg)
        

            #go to parking
            self.go_turn_2 = False
            self.park = True

    def parking(self):
        if(self.park):
            PI = 3.1415926535897
            self.get_logger().info('PARKING')
            #move forward during some time
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            t1 = self.get_clock().now().nanoseconds * (10**(-9))
            moving_seconds = 1
            vel_msg = Twist()
            vel_msg.linear.x = 0.4
            while((t1-t0)<moving_seconds):
                self.publisher.publish(vel_msg)

            vel_msg.linear.x = 0
            self.publisher.publish(vel_msg)

            #wait
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            t1 = self.get_clock().now().nanoseconds * (10**(-9))
            waiting_seconds = 2
            vel_msg.linear.x = 0
            while((t1-t0)<waiting_seconds):
                self.publisher.publish(vel_msg)

            #move back during some time
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            t1 = self.get_clock().now().nanoseconds * (10**(-9))
            moving_seconds = 1
            vel_msg.linear.x = -0.4
            while((t1-t0)<moving_seconds):
                self.publisher.publish(vel_msg)

            vel_msg.linear.x = 0
            self.publisher.publish(vel_msg)
            self.park = False
            self.go_back = True

    def back(self):
        if (self.go_back):
            PI = 3.1415926535897
            self.get_logger().info('GOING BACK')
            #turn right\left depends on self.turned_right
            vel_msg = Twist()
            angle = 90
            speed = 0.4
            angular_speed = speed*2*PI/360
            relative_angle = angle*2*PI/360
            if (self.turned_right):
                vel_msg.angular.z = -abs(angular_speed)                
            else:
                vel_msg.angular.z = abs(angular_speed)
            #ros1 = Time.now().to_sec(), ROS2 - ????
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            current_angle = 0
            while(current_angle < relative_angle):
                self.publisher.publish(vel_msg)
                t1 = self.get_clock().now().nanoseconds * (10**(-9))
                current_angle = angular_speed*(t1-t0)

            vel_msg.angular.z = 0
            self.publisher.publish(vel_msg)

            #go backward
            #during some time: vel_msg.linear.x = -0.4
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            t1 = self.get_clock().now().nanoseconds * (10**(-9))
            moving_seconds = 21
            vel_msg.linear.x = -0.4
            while((t1-t0)<moving_seconds):
                self.publisher.publish(vel_msg)

            vel_msg.linear.x = 0
            self.publisher.publish(vel_msg)

            #turn right 90 degrees
            vel_msg.angular.z = abs(angular_speed)
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            current_angle = 0
            while(current_angle < relative_angle):
                self.publisher.publish(vel_msg)
                t1 = self.get_clock().now().nanoseconds * (10**(-9))
                current_angle = angular_speed*(t1-t0)

            vel_msg.angular.z = 0
            self.publisher.publish(vel_msg)

            #go forward a little to find yellow and white lines of road
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            t1 = self.get_clock().now().nanoseconds * (10**(-9))
            moving_seconds = 1
            vel_msg.linear.x = 0.4
            while((t1-t0)<moving_seconds):
                self.publisher.publish(vel_msg)

            vel_msg.linear.x = 0
            self.publisher.publish(vel_msg)

            #send meaasge that mission is complete, turn on lane_follower
            self.go_back = False
            self.is_mission_complete = True

    def end_of_mission(self):
        if(self.is_mission_complete):
            self.get_logger().info('Mission is complete :3')
            subprocess.call(["ros2", "topic", "pub", "--once", "/state", "std_msgs/String", "'data: follow'"])
            self.is_mission_complete = False

                




        




def main(args=None):
    rclpy.init(args=args)
    node = Laser_pepe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()