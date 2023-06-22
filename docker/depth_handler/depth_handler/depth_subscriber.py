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
        self.distance_pub = self.create_publisher(Float64, 'obstacle_distance', 10)
        self.depth_sub = self.create_subscription(
            Image,
            'camera1/depth/image_rect_raw',
            self.get_nearest_point,
            10)

    def get_nearest_point(self, msg):
        self.get_logger().info('Image received')
        try:
            depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_array = np.array(depth_image, dtype=np.float32)
            warning_msg = Float64()
            min_distance = float(np.min([np.nonzero(depth_array)]))
            warning_msg.data = min_distance
            self.get_logger().info(f'Min distance: {min_distance}')
            self.distance_pub.publish(warning_msg)
            
                

        except CvBridgeError as e:
            print(e)


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