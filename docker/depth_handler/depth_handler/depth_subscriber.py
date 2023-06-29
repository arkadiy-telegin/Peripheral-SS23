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
        self.min_dis1 = None
        self.min_dis2 = None
        self.distance_pub = self.create_publisher(Float64, 'obstacle_distance', 1)
        self.im_pub = self.create_publisher(Image, 'warn_image', 1)
        # self.depth_sub = self.create_subscription(
        #     Image,
        #     'camera1/depth/image_rect_raw',
        #     self.get_nearest_point_cam1,
        #     3)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera2/depth/image_rect_raw',
            self.get_nearest_point_cam2,
            3)
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera2/color/image_raw',
            self.rgb_cam2,
            3)


    def get_nearest_point_cam1(self, msg):
        # self.get_logger().info('Image received')
        try:
            depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_array = np.array(depth_image, dtype=np.float32)
            self.min_dis1 = float(np.min(depth_array[depth_array!=0]))

        except CvBridgeError as e:
            print(e)

        
    def get_nearest_point_cam2(self, msg):
        # self.get_logger().info('Image received')
        try:
            depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # self.denoising(depth_image)
            depth_array = np.array(depth_image, dtype=np.float64)
            warning_msg = Float64()
            self.min_dis2 = float(np.min(depth_array[depth_array!=0]))
            # self.get_logger().info(f'Min distance camera 1: {self.min_dis2}')

            if self.min_dis2 != None:
                # self.min_dis = min(self.min_dis1, self.min_dis2)
                self.min_dis = self.min_dis2
                warning_msg.data = self.min_dis
                # self.get_logger().info(f'Min distance of both cameras: {self.min_dis}')
                self.distance_pub.publish(warning_msg)
            
        except CvBridgeError as e:
            print(e)


    def denoising(self, image):
        fixup = image==0
        print(fixup)
        denoised_image = image.copy()
        denoised_image[fixup] = np.mean(image)
        self.im_pub.publish(bridge.cv2_to_imgmsg(denoised_image))

    def rgb_cam2(self, msg):
        if self.min_dis2 != None and self.min_dis2 < 300:
            rgb_arr = bridge.imgmsg_to_cv2(msg)
            rgb_arr[:,:,2] = rgb_arr[:,:,2] + 50
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