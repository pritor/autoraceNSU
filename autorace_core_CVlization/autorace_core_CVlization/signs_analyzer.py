import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
import os
from ament_index_python.packages import get_package_share_directory

class ImageAnalyzer(Node):
    def __init__(self, signs):
        super().__init__('image_analyzer')
        self.signs = signs
        self.img_sub = self.create_subscription(Image, '/color/image', self.img_callback, 1)
        self.st_pub = self.create_publisher(String, '/state', 1)
        self.dbg_pub = self.create_publisher(Image, '/debug/image', 1)
        self.br = CvBridge()
        self.started = False
        self.sift= cv2.SIFT_create()
        self.bf = cv2.BFMatcher()

    def img_callback(self, msg):
        image = self.br.imgmsg_to_cv2(msg, "bgr8")
        res = self.analyze(image)
        res_msg = String()
        res_msg.data = res
        self.st_pub.publish(res_msg)
        # self.get_logger().info("result of image analyzing is "+res)

    def analyze(self, image):
        if self.started:
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ikp, ides= self.sift.detectAndCompute(image_gray, None)
            # BFMatcher решает матч
            for i in self.signs:
                matches = self.bf.knnMatch(i[1], ides, k=2)
                # Отрегулируйте коэффициент
                good = []
                for m, n in matches:
                    if m.distance < 0.55 * n.distance:
                        good.append([m])
                # sign_pth = os.path.join(get_package_share_directory('autorace_core_CVlization'), 'signs', 'tunnel.png')
                # sign = cv2.imread(sign_pth)
                # dbg = cv2.drawMatchesKnn(sign, i[0], image, ikp, good, None, flags=2)
                # msg = self.br.cv2_to_imgmsg(dbg, 'bgr8')
                # self.dbg_pub.publish(msg)
                if len(good) >= 20:
                    # self.get_logger().info("result of image analyzing is " + "found "+i[2])
                    return i[2]
        else:
            # self.started =True

            lower_green= np.array([0, 100, 0])
            upper_green= np.array([5, 120, 5])
            mask = cv2.inRange(image, lower_green, upper_green)
            fraction = np.count_nonzero(mask)
            if fraction > 1000:
                self.get_logger().info('green found')
                self.started = True
                return 'follow'
            # res = cv2.bitwise_and(image, image, mask=mask)
            #
            # msg = self.br.cv2_to_imgmsg(res, 'bgr8')
            # self.dbg_pub.publish(msg)

        return "nothing"



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
    signs = generate_keypoints(['intersection.png', 'construction.png',"parking.png","pedestrian.png",'tunnel.png'])
    rclpy.init(args=args)
    node = ImageAnalyzer(signs)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()
