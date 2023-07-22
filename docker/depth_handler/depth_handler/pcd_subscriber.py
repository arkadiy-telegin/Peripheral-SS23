import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, JointState
import sensor_msgs_py.point_cloud2 as pc2

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
bridge = CvBridge()


class DepthSubscriber(Node):

    def __init__(self):

        super().__init__('pcd_subscriber')

        self.threshold = 0.4
        self.min_threshold = 0.1
        self.a,self.b,self.c,self.d = self.parametrize_plane()

        self.pcd_pub = self.create_publisher(PointCloud2, 'warn_pcd', 1)
        self.env_pub = self.create_publisher(PointCloud2, 'env_pcd', 1)
        self.arm_pub = self.create_publisher(PointCloud2, 'arm_pcd', 1)
        self.depth_sub = self.create_subscription(
            PointCloud2,
            '/camera2/depth/color/points',
            self.get_close_points_to_plane,
            3)


    def get_close_points_to_plane(self, msg):

        # self.get_logger().info('Image received')
        try:
            self.points = pc2.read_points_numpy(msg, skip_nans=True, field_names=("x", "y", "z"))
            self.points = self.points[self.points[:,2]<0.7]

            self.calc_distance()
            mask = self.distances < self.threshold 
            mask_min = self.min_threshold < self.distances
            indices = np.where(mask)
            indices_min = np.where(mask_min)
            pub_indices = np.intersect1d(indices, indices_min)

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera2_link'
            if self.points[pub_indices,:].size != 0:
                pc_pub = pc2.create_cloud_xyz32(header, self.points[pub_indices,:])
                self.pcd_pub.publish(pc_pub)

        except CvBridgeError as e:
            print(e)
        
    def parametrize_plane(self):
        v1 = [0,1,0]
        v2 = [1,0,0]
        # v2 = [np.sin(50), 0, np.cos(50)]
        point = [0,0,0]
        a,b,c = np.cross(v1, v2)
        d = np.dot([a,b,c], point)
        return a,b,c,d

    def calc_distance(self):
        norm = np.sqrt(self.a**2 + self.b**2 + self.c**2)
        numerator = np.abs(np.subtract(self.points.dot([self.a, self.b, self.c]), self.d))
        self.distances = numerator/norm

    def get_close_points_to_line(self, msg):
        radius = 0.1 # radius of the arm 

        # calculate position of shoulder in camera position
        alpha = 50 # angle at which the camera is tilted in relation to floor
        p = [1,0,1]
        r = [[np.cos(alpha), 0, np.sin(alpha)],
            [0,1,0],
            [-np.sin(alpha), 0, np.cos(alpha)]]
        p = r*p

        ## Parametrize the upper arm with initial point and vector n
        theta = 0.1 # subscribe to joint
        phi = 0.1 
        arm_d = 0.5 # length of the upper arm
        n = d*[np.sin(theta),np.cos(theta)*np.cos(phi), np.sin(phi)]
        n = n / np.linalg.norm(n)

        ## Calculate distance to that line segment
        p1 = np.substract(self.points, p)
        t = np.dot(p1, n)
        t[t>arm_d] = arm_d
        p_closest = p + np.outer(t*n)
        d = np.sqrt(np.sum(np.square(self.points - p_closest)))

        ## parametrize forearm
        elbow_d = 0.3
        p_u = p + t*n
        elbow_angle = 0.1
        r2 = [[np.cos(elbow_angle), 0, np.sin(elbow_angle)],
              [0,1,0],
              [-np.sin(elbow_angle), 0, np.cos(elbow_angle)]]
        n2 = np.dot(r2, n)
        p2 = np.subtract(self.points, p1)
        t2 = np.dot(p1, n)
        t2[t2>elbow_d] = elbow_d
        p2_closest = p_u + np.outer(t2*n2)
        d2 = np.sqrt(np.sum(np.square(self.points - p2_closest)))

        mask1 = np.where(d < radius)
        mask2 = np.where(d2 < radius)
        mask = np.intersect1d(mask1, mask2)
        outer_mask1 = np.where(radius < d)
        outer_mask2 = np.where(radius < d2)
        outer_mask = np.intersect1d(outer_mask1, outer_mask2)

        count = 0
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera2_link'
        if self.points[mask,:].size != 0:
                arm_pub = pc2.create_cloud_xyz32(header, self.points[mask,:])
                self.arm_pub.publish(arm_pub)
        if self.points[outer_mask,:].size != 0:
            env_pcd = pc2.create_cloud_xyz32(header, self.points[outer_mask,:])
            self.env_pub.publish(env_pcd)




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