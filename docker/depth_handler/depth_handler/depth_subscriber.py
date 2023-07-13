import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
bridge = CvBridge()


class DepthSubscriber(Node):

    def __init__(self):

        super().__init__('minimal_subscriber')
        self.min_dis = None
        self.distance_pub = self.create_publisher(Float64, 'obstacle_distance', 1)
        self.im_pub = self.create_publisher(Image, 'warn_image', 1)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera1/depth/image_rect_raw',
            self.get_nearest_point,
            3)
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera1/color/image_raw',
            self.rgb_cam,
            3)


    def get_nearest_point(self, msg):

        # self.get_logger().info('Image received')
        try:
            depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_array = np.array(depth_image, dtype=np.float64)
            warning_msg = Float64()
            self.min_dis = float(np.min(depth_array[depth_array!=0]))
            # self.get_logger().info(f'Min distance camera 1: {self.min_dis2}')

            if self.min_dis != None:
                self.min_dis = self.min_dis
                warning_msg.data = self.min_dis
                # self.get_logger().info(f'Min distance of both cameras: {self.min_dis}')
                self.distance_pub.publish(warning_msg)
            
        except CvBridgeError as e:
            print(e)

    def rgb_cam(self, msg):


        rgb_arr = bridge.imgmsg_to_cv2(msg)
        red_img  = np.full(np.shape(rgb_arr), (0,0,255), np.uint8)
        # if self.min_dis != None and self.min_dis < 200:
        #     rgb_arr = cv.addWeighted(rgb_arr, 0.4, red_img, 0.6, 0)
        #     self.rgb_image = bridge.cv2_to_imgmsg(rgb_arr)

        # elif self.min_dis != None and self.min_dis < 300:
        #     rgb_arr = cv.addWeighted(rgb_arr, 0.6, red_img, 0.4, 0)
        #     self.rgb_image = bridge.cv2_to_imgmsg(rgb_arr)

        # elif self.min_dis != None and self.min_dis < 400:
        #     rgb_arr = cv.addWeighted(rgb_arr, 0.8, red_img, 0.2, 0)
        #     self.rgb_image = bridge.cv2_to_imgmsg(rgb_arr)
        if self.min_dis != None and self.min_dis < 400:
            weight = -3*self.min_dis/1000 + 1.2
            rgb_arr = cv.addWeighted(rgb_arr, 1-weight, red_img, weight, 0)
            self.rgb_image = bridge.cv2_to_imgmsg(rgb_arr)
            
        else:
            self.rgb_image = msg
            
        self.im_pub.publish(self.rgb_image)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DepthSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()