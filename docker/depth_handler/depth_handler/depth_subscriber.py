import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()


class DepthSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.depth_sub = self.create_subscription(
            String,
            'camera1/depth/image_rect_raw',
            self.get_nearest_point,
            10)
        self.distance_pub = self.create_publisher(Float64, 'obstacle_distance', 10)

    def get_nearest_point(self, msg):
        self.get_logger().info('Image received')
        try:
            depth_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            warning_msg = Float64()

            min_distance = min(depth_image)
            warning_msg.data = min_distance
            self.get_logger().info('Minimal distance', min_distance)
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