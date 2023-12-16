import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from rclpy.parameter import Parameter

import os
from ament_index_python.packages import get_package_share_directory


class IntersectionNode(Node):
    def __init__(self, signs):
        super().__init__('intersection')

        self.sub_image_original = self.create_subscription(Image, '/color/image', self.cbChooseDirection, 10)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cbLidar, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.dbg_pub = self.create_publisher(Image, '/debug/image', 1)
        self.st_pub = self.create_publisher(String, '/state', 1)
        self.shtdwn_pub = self.create_publisher(Bool, '/shutdown/lane_follower', 1)

        self.started = False
        self.time_to_stop = False
        self.turned = False
        self.direction = "none"

        self.signs = signs
        self.br = CvBridge()
        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher()

        self.timer_node = rclpy.create_node('intersection_timer')
        self.timer_node.set_parameters([Parameter('use_sim_time', value=True)])
        self.timer = self.timer_node.create_timer(0.0001, self.timerCb)
        self.cur_time = 0

    def timerCb(self):
        # self.get_logger().info('timer_node callback')
        self.cur_time = self.timer_node.get_clock().now().nanoseconds * 1e-9

    def sleep(self, secs):
        rclpy.spin_once(self.timer_node)  # two times, because one is not enough to update cur_time
        rclpy.spin_once(self.timer_node)
        t0 = self.cur_time
        d = 0
        # self.get_logger().info(f'timer started with {t0} and {secs}')
        while d <= secs:
            rclpy.spin_once(self.timer_node)
            t = self.cur_time
            d = t - t0
        # self.get_logger().info(f'timer ended with {t}')

    def cbChooseDirection(self, msg):
        if not self.started:
            image = self.br.imgmsg_to_cv2(msg, "bgr8")
            res = self.analyze(image)
            if res != 'nothing':
                self.direction = res
                self.started = True
        else:
            if self.time_to_stop and not self.turned:

                shtdwn = Bool()
                shtdwn.data = True
                self.shtdwn_pub.publish(shtdwn)
                self.sleep(1.5)
                if self.direction == 'left':
                    vel = Twist()
                    vel.linear.x = 0.03
                    vel.angular.z = 0.5
                    self.pub_cmd_vel.publish(vel)
                    self.sleep(4.2)
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    self.pub_cmd_vel.publish(vel)
                elif self.direction == 'right':
                    vel = Twist()
                    vel.linear.x = 0.05
                    self.pub_cmd_vel.publish(vel)

                state = String()
                state.data = 'follow'
                self.turned = True
                self.st_pub.publish(state)
                self.sleep(2)
                state = String()
                state.data = 'completed'
                self.st_pub.publish(state)
                self.timer_node.destroy_node()
                self.destroy_node()

    def cbLidar(self, msg):
        lidar = np.array(msg.ranges)
        min_i = np.argmin(lidar)
        min = lidar[min_i]
        if self.started and not self.time_to_stop:
            if min < 35 and 20 < min_i < 110:
                self.time_to_stop = True
                self.get_logger().info('stop condition')
        # elif self.started and self.turned:
        #     if 30 < lidar[0] < 50:
        #         self.get_logger().info('turn off condition')
        #         state = String()
        #         state.data = 'completed'
        #         self.st_pub.publish(state)
        #         self.destroy_node()

    def analyze(self, image):
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ikp, ides = self.sift.detectAndCompute(image_gray, None)
        # BFMatcher решает матч
        for i in self.signs:
            matches = self.bf.knnMatch(i[1], ides, k=2)
            # Отрегулируйте коэффициент
            good = []
            for m, n in matches:
                if m.distance < 0.50 * n.distance:
                    good.append([m])
            # sign_pth = os.path.join(get_package_share_directory('autorace_core_CVlization'), 'signs', 'left.png')
            # sign = cv2.imread(sign_pth)
            # dbg = cv2.drawMatchesKnn(sign, i[0], image, ikp, good, None, flags=2)
            # msg = self.br.cv2_to_imgmsg(dbg, 'bgr8')
            # self.dbg_pub.publish(msg)
            if len(good) >= 10:
                self.get_logger().info("intersection image analyzing " + "found " + i[2])
                return i[2]
        return 'nothing'


def generate_keypoints(pngs):
    sift = cv2.SIFT_create()
    kps = []
    for i in pngs:
        sign_pth = os.path.join(get_package_share_directory('autorace_core_CVlization'), 'signs', i)
        sign = cv2.imread(sign_pth)
        sign_gray = cv2.cvtColor(sign, cv2.COLOR_BGR2GRAY)
        skp, sdes = sift.detectAndCompute(sign_gray, None)
        name = i.split('.')[0]
        kps.append((skp, sdes, name))
    return kps


def main(args=None):
    signs = generate_keypoints(['left.png', 'right.png'])
    rclpy.init(args=args)
    node = IntersectionNode(signs)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
