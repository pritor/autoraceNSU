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
from rclpy.parameter import Parameter
import time
from std_msgs.msg import Bool
from std_msgs.msg import String


class Laser_pepe(Node):

    def __init__(self):
        super().__init__('laser_pepe')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #ros2 topic pub --once /shutdown/lane_follower std_msgs/Bool 'data: True'
        self.stop_lane_follower_publisher = self.create_publisher(Bool, '/shutdown/lane_follower', 10)
        self.run_lane_follower_publisher = self.create_publisher(String, '/state', 10)
        self.subscriber_2 = self.create_subscription(LaserScan, '/scan', self.hard_code, 10)
        self.scan = LaserScan()
        #self.timer = self.create_timer(0.1, self.move)
        self.begin = True
        self.go_turn_1 = False
        self.go_turn_2 = False
        self.go_turn_3 = False
        self.park = False
        self.go_back = False
        self.turn_back = False
        self.go_back_1 = False
        self.turn_back_1 = False
        self.turned_right = False
        self.is_mission_complete = False
        self.control_index = 0
        self.flag_pepe = True

        self.sing_index_back = 0
        self.car_index_before_turning = 0
        self.distance_to_car = 0.0


        self.timer_node = rclpy.create_node('parking_timer')
        self.timer_node.set_parameters([Parameter('use_sim_time', value=True)])
        self.timer = self.timer_node.create_timer(0.0001, self.timerCb)
        self.cur_time = 0

        self.get_logger().info('laser_pepe initialized')

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

    def hard_code(self, msg):
        PI = 3.1415926535897
        condition = Bool()
        flag = True
        laser = np.array(msg.ranges)
        min_val = min(laser)
        index_min_val = np.argmin(laser)
        if  (self.begin):
            #robot is moving on line_folower пока не окажется на перекрестке, останавливается
            #ranges = [218:235+5?]
            self.get_logger().info(f'STARTED MISSION1 {self.begin}')
            value_max = 0
            value_min = 0

            self.get_logger().info('min index is ' + str(index_min_val))
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            if (index_min_val>220 and index_min_val<238):
                self.control_index = index_min_val
                flag = False
                stop_msg = Bool()        
                stop_msg.data = True
                self.stop_lane_follower_publisher.publish(stop_msg)
                time.sleep(1)

                self.begin = False
                self.go_turn_1 = True
                self.get_logger().info(f'ready to turn left {self.begin}')

        if (self.go_turn_1):
            if (self.begin):
                self.get_logger().info('pupupu')
                #self.begin = False
                self.go_turn_1 = True

            self.get_logger().info('starting turn left')

            PI = 3.1415926535897
            #stop line_finder

            #turn on 90o
            

            if (self.flag_pepe ):
                self.control_index = index_min_val
                self.flag_pepe = False

            angle = 60
            speed = 0.4
            angular_speed = speed*2*PI/360
            relative_angle = (angle*2*PI/360)/10
            vel_msg = Twist()
            vel_msg.angular.z = abs(angular_speed*110)
            #ros1 = Time.now().to_sec(), ROS2 - ????
            t0 = self.get_clock().now().nanoseconds * (10**(-9))
            current_angle = 0
            #self.control_index + 90
            #131 = 125

            #if (index_min_val > (140)):
            #    self.get_logger().info(f'TURNING LEFT {index_min_val } {140}')
            #    self.publisher.publish(vel_msg)
            #    t1 = self.get_clock().now().nanoseconds * (10**(-9))
            #    current_angle = angular_speed*((t1-t0))
            #else:
            #    self.get_logger().info(f'ready to go forward {index_min_val } {140}')
            #    vel_msg.angular.z = 0.0
            #    self.publisher.publish(vel_msg)
            #    if (self.begin):
            #        self.get_logger().info('pupupu after stopped')
            #        self.begin = False

                #go to find_place
            #    self.go_turn_1 = False
            #    self.go_turn_2 = True

            self.publisher.publish(vel_msg)
            self.sleep(2.05)
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)

            self.go_turn_1 = False
            self.go_turn_2 = True

        if(self.go_turn_2 ):
            
            
            self.get_logger().info(f'going forward, index {index_min_val} val {min_val}')
            
            angle = 90
            speed = 0.4
            angular_speed = speed*2*PI/360
            relative_angle = angle*2*PI/360
            vel_msg = Twist()
            #during some time: vel_msg.linear.x = 0.4
            
            vel_msg.linear.x = 0.27

            #car on the Right +-256, car on the Left +-76
            if (index_min_val>75 and index_min_val<108):
                if (self.begin):
                    self.get_logger().info('begin blocked r')
                    #self.begin = False

                vel_msg.linear.x = 0.0
                self.publisher.publish(vel_msg)
                self.car_index_before_turning = index_min_val
                self.get_logger().info(f' stopped R')
                time.sleep(1)

                #car on the left side, turn right
                #send flag turn Left
                self.turned_right = True
                #go to parking
                self.go_turn_2 = False
                self.go_turn_3 = True


                
            elif (index_min_val>255 and index_min_val<288):
                if (self.begin):
                    self.get_logger().info('begin blocked l')
                    s#elf.begin = False

                vel_msg.linear.x = 0.0
                self.publisher.publish(vel_msg)

                self.car_index_before_turning = index_min_val
                self.get_logger().info(f' stopped L')
                time.sleep(1)


                #car on the right side, turn left
                self.turned_right = False
                #send flag turn Left
                #go to parking
                self.go_turn_2 = False
                self.go_turn_3 = True

            else:
                if (self.begin):
                    self.get_logger().info('begin blocked F')
                    self.begin = False
                #go forward
                self.publisher.publish(vel_msg)
                
        #STOPPED HERE turn to parking
        if (self.go_turn_3 ):
            
            self.get_logger().info(f'turning to free parking lot, index {index_min_val} val {min_val}')
            val = 0.11

            angle = 90
            speed = 0.15
            angular_speed = speed*2*PI/360
            relative_angle = angle*2*PI/360
            vel_msg = Twist()

            if (min_val < val):
                vel_msg_cor = Twist()
                vel_msg.angular.z = 0.0
                vel_msg_cor.linear.x = 0.1
                self.publisher.publish(vel_msg)
                self.sleep(0.1)
                vel_msg_cor.linear.x = 0.0
                self.publisher.publish(vel_msg)

            if(self.turned_right):
                #need to turn RIGHT
                vel_msg.angular.z = -abs(angular_speed*100)
                condition = index_min_val>179
            else:
                #need to turn LEFT
                vel_msg.angular.z = abs(angular_speed*100)
                condition = index_min_val >179 and index_min_val <187 

            




            if(condition):
                #car is on robot's back
                vel_msg.angular.z = 0.0
                self.publisher.publish(vel_msg)
                self.distance_to_car = min_val

                self.go_turn_3 = False
                self.park = True
            else:
                self.publisher.publish(vel_msg)

            #self.publisher.publish(vel_msg)
            #self.sleep(3.2)
            #vel_msg.angular.z = 0.0
            #self.publisher.publish(vel_msg)
            #self.go_turn_3 = False
            #self.park = True


        if(self.park):
            if (self.begin):
                self.get_logger().info(f'begin blocked before parking, index {index_min_val} val {min_val}')
                self.begin = False
            self.get_logger().info(f'PARKING, index {index_min_val} val {min_val}')

            
            
            #move forward during some time
            vel_msg = Twist()
            vel_msg.linear.x = 0.10
            self.publisher.publish(vel_msg)

            #wait
            
            if (min_val > 0.42):
                vel_msg.linear.x = 0.0
                self.publisher.publish(vel_msg)
                self.park = False
                self.get_logger().info('waiting 1 sec before going back...')
                self.sleep(1)
                self.go_back = True



        if (self.go_back):
            PI = 3.1415926535897
            self.get_logger().info('GOING BACK to car')
            #turn right\left depends on self.turned_right
            vel_msg = Twist()
            #move back
            #val 0.14938101172447205
            vel_msg.linear.x = -0.10

            if (min_val>self.distance_to_car):
                self.publisher.publish(vel_msg)
            else:
                vel_msg.linear.x = 0.0
                self.publisher.publish(vel_msg)
                self.go_back = False
                self.turn_back = True

        if(self.turn_back):
            self.get_logger().info('turn to previous condition near car')
            #razvorot
            vel_msg = Twist()
            angle = 90
            speed = 0.4
            angular_speed = speed*2*PI/360
            relative_angle = angle*2*PI/360

            left = 260
            right = 86

            if (self.turned_right):
                vel_msg.angular.z = abs(angular_speed*100)
                pepe = right               
            else:
                vel_msg.angular.z = -abs(angular_speed*100)
                pepe = left
            #ros1 = Time.now().to_sec(), ROS2 - ????
            if ((index_min_val > (pepe-5)) and (index_min_val < (pepe + 5))):
                vel_msg.angular.z = 0.0
                self.publisher.publish(vel_msg)
                self.turn_back = False
                self.go_back_1 = True
            else:
                self.get_logger().info(f'index {index_min_val} to {pepe}')
                self.publisher.publish(vel_msg)
            #self.publisher.publish(vel_msg)
            #self.sleep(2)
            #vel_msg.angular.z = 0.0
            #self.publisher.publish(vel_msg)
            #self.turn_back = False
            #self.go_back_1 = True



        if (self.go_back_1):
            self.get_logger().info(f'go backward to sign, index {index_min_val} val {min_val}')
            vel_msg = Twist()
            vel_msg.linear.x = -0.10
            self.publisher.publish(vel_msg)
            self.sleep(9)
            vel_msg.linear.x = 0.0
            self.publisher.publish(vel_msg)


            self.go_back_1 = False
            self.turn_back_1 = True

        if (self.turn_back_1):
            self.get_logger().info('turn right and return to main road')
            angle = 60
            speed = 0.4
            angular_speed = speed*2*PI/360
            relative_angle = (angle*2*PI/360)/10
            vel_msg = Twist()
            vel_msg.angular.z = -abs(angular_speed*110)
            self.publisher.publish(vel_msg)
            self.sleep(2.05)
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg)   

            vel_msg = Twist() 
            vel_msg.linear.x = 0.1
            self.publisher.publish(vel_msg)
            self.sleep(1)
            vel_msg.linear.x = 0.0
            self.publisher.publish(vel_msg)          

            #send meaasge that mission is complete, turn on lane_follower
            self.turn_back_1 = False
            self.is_mission_complete = True

        if(self.is_mission_complete):
            self.get_logger().info('parking mission done')
            run_msg = String()
            run_msg.data = 'follow'
            self.run_lane_follower_publisher.publish(run_msg)
            time.sleep(1)
            run_msg.data = 'completed'
            self.run_lane_follower_publisher.publish(run_msg)
            self.destroy_node()
            self.is_mission_complete = True

                




        




def main(args=None):
    rclpy.init(args=args)
    node = Laser_pepe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()