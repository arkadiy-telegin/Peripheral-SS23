import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
bridge = CvBridge()


class DepthSubscriber(Node):

    def __init__(self):

        super().__init__('pcd_subscriber')

        self.threshold = 0.3
        self.a,self.b,self.c,self.d = self.parametrize_plane()

        self.pcd_pub = self.create_publisher(PointCloud2, 'warn_pcd', 1)
        self.depth_sub = self.create_subscription(
            PointCloud2,
            '/camera1/depth/color/points',
            self.get_close_points,
            3)


    def get_close_points(self, msg):

        # self.get_logger().info('Image received')
        try:
            self.points = pc2.read_points_numpy(msg, skip_nans=True, field_names=("x", "y", "z"))
            self.points = self.points[self.points[:,0]<1.2]

            self.calc_distance()
            mask = self.distances < self.threshold
            indices = np.where(mask)

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera1_link'
            if self.points[indices,:].size != 0:
                pc_pub = pc2.create_cloud_xyz32(header, self.points[indices,:])
                self.pcd_pub.publish(pc_pub)

        except CvBridgeError as e:
            print(e)
        
    def parametrize_plane(self):
        v1 = [0,1,0]
        v2 = [1,0,0]
        point = [0,0,0]
        a,b,c = np.cross(v1, v2)
        d = np.dot([a,b,c], point)
        return a,b,c,d

    def calc_distance(self):
        norm = np.sqrt(self.a**2 + self.b**2 + self.c**2)
        numerator = np.abs(np.subtract(self.points.dot([self.a, self.b, self.c]), self.d))
        self.distances = numerator/norm


def main(args=None):
    rclpy.init(args=args)

    pcd_subscriber = DepthSubscriber()

    rclpy.spin(pcd_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()