import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, JointState
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker

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
        self.rot_x = None
        self.rot_y = None
        self.rot_z = None
        self.elbow_angle = None

        self.pcd_pub = self.create_publisher(PointCloud2, 'warn_pcd', 1)
        self.env_pub = self.create_publisher(PointCloud2, 'env_pcd', 1)
        self.arm_pub = self.create_publisher(PointCloud2, 'arm_pcd', 1)
        self.marker_pub = self.create_publisher(Marker, 'marker', 1)
        self.depth_sub = self.create_subscription(
            PointCloud2,
            '/camera2/depth/color/points',
            self.get_close_points_to_line,          # If you want to parametrize arm with plane, use self.get_close_point_to_plane
            3)
        self.jointstatesub = self.create_subscription(
            JointState,
            '/roboy/pinky/control/cardsflow_joint_states',
            self.get_joint_states,
            3)


    def get_close_points_to_plane(self, msg):
        # Use this if zou want to parametrize arm with a plane
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
        v1 = [1,0,0]
        v2 = [0,1,0]
        # v2 = [0, np.sin(50), np.cos(50)]
        point = [0,0,0]
        a,b,c = np.cross(v1, v2)
        d = np.dot([a,b,c], point)
        return a,b,c,d

    def calc_distance(self):
        # Calculate the distance from each point to the plane parametrized by vectors v1,v2
        norm = np.sqrt(self.a**2 + self.b**2 + self.c**2)
        numerator = np.abs(np.subtract(self.points.dot([self.a, self.b, self.c]), self.d))
        self.distances = numerator/norm

    def get_close_points_to_line(self, msg):
        # Use this if you want to parametrize arm with two lines --> default, more exact

        radius_arm = 0.125
        radius_forearm = 0.075
        elbow_d = 0.46   # Length of the forearm
        arm_d = 0.4   # Length of the upper arm
        alpha = 50 # angle at which the camera is tilted in relation to floor

        angle = alpha*np.pi/180
        rot_camera = np.array([[1,0,0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]])
        
        # calculate position of shoulder joint in camera position; shoulder joint is not coincident with camera CS
        p = np.array([0.04, 0.07, -0.05])
        p = np.matmul(rot_camera,p)

        # Read points
        self.points = pc2.read_points_numpy(msg, skip_nans=True, field_names=("x", "y", "z"))
        # Rotate the to world coordinate frame
        self.points = self.points @ rot_camera
        # Remove points belonging to floor
        self.points = self.points[self.points[:,1]<1.10]



        ## Parametrize the upper arm with initial point and vector n 
        if self.rot_x is not None and self.rot_y is not None and self.rot_z is not None:
            rot_x = np.array([[1,0,0],
                              [0,np.cos(self.rot_x),-np.sin(self.rot_x)],
                              [0, np.sin(self.rot_x), np.cos(self.rot_x)]])
            rot_y = np.array([[np.cos(self.rot_y),0,np.sin(self.rot_y)],
                              [0,1,0],
                              [-np.sin(self.rot_y),0,np.cos(self.rot_y)]])
            rot_z = np.array([[np.cos(self.rot_z),-np.sin(self.rot_z),0],
                              [np.sin(self.rot_z),np.cos(self.rot_z),0],
                              [0,0,1]])
        else:
            self.get_logger().info('Could not get joint values, set shoulder joint angles to zero')
            rot_x = np.eye(3)
            rot_y = np.eye(3)
            rot_z = np.eye(3)
        
        # Normal vector changes with joint values
        n = rot_z @ rot_y @ rot_x @ np.array([0,1,0])

        # Calculate distance to that line segment
        p1 = np.subtract(self.points, p)
        t = np.dot(p1, n)
        t[np.abs(t)>arm_d] = arm_d
        p_closest = p + np.multiply(np.repeat(t[:, None], 3, axis=1),n)
        d = np.sqrt(np.sum(np.square(self.points - p_closest), axis=1))

        # parametrize forearm, same as above
        p_u = p + arm_d*n
        n_elbow = np.array([0,0.36,0])
        if self.elbow_angle is not None:
            rot_elbow = np.array([[np.cos(self.elbow_angle),-np.sin(self.elbow_angle),0],
            [np.sin(self.elbow_angle),np.cos(self.elbow_angle),0],
            [0,0,1]])
        else:
            self.get_logger().info('Could not get joint values, set elbow joint angle to zero')
            rot_elbow = np.eye(3)
        n_elbow = np.dot(rot_elbow, n)

        # Calculate distance from elbow to each joint
        p2 = np.subtract(self.points, p_u)
        t2 = np.dot(p2, n)
        t2[t2>elbow_d] = elbow_d
        t2[t2<0] = 100
        p2_closest = p_u + np.multiply(np.repeat(t2[:, None], 3, axis=1),n_elbow)
        d2 = np.sqrt(np.sum(np.square(self.points - p2_closest), axis=1))

        # Create point cloud showing the arm
        num_steps = 20
        angular_steps = 30
        arm_points = []

        # Find perpendicular vector to n
        v = np.array([n[0]+1, n[1]+2, n[2]])
        v_p = np.cross(v, n)
        v_p_list = []
        # Same for elbow
        v_e = np.array([n_elbow[0]+1, n_elbow[1]+2, n_elbow[2]])
        v_e_p = np.cross(v_e, n_elbow)
        v_e_p_list = []

        # Idea: Rotate a vector of lenght radius perpendicular to normal,
        # at end of each vector add a point
        for j in range(angular_steps):
                # Roddrigues formula for rotation of vector v_p around vector n
            v_p_list.append(v_p * np.cos(np.pi*2/angular_steps*(j+1)) + np.cross(n, v_p) * np.sin(np.pi*2/angular_steps*(j+1)) + n * np.dot(n,v_p) * (1-np.cos(np.pi*2/angular_steps*(j+1))))
            v_e_p_list.append(v_e_p * np.cos(np.pi*2/angular_steps*(j+1)) + np.cross(n, v_e_p) * np.sin(np.pi*2/angular_steps*(j+1)) + n * np.dot(n,v_e_p) * (1-np.cos(np.pi*2/angular_steps*(j+1))))
        for i in range(num_steps):
            p_current = p + float(arm_d/num_steps*i)*n
            p_e_current = p_u + float(elbow_d/num_steps*i)*n_elbow
            arm_points.extend(p_current+v_p_list/np.linalg.norm(v_p)*radius_arm)
            arm_points.extend(p_e_current + v_e_p_list/np.linalg.norm(v_e_p)*radius_forearm)

        # # points belonging to the arm
        # mask1 = np.where(d < radius_arm)
        # mask2 = np.where(d2 < radius_forearm)
        # mask = np.concatenate([mask1, mask2], axis=1)

        # Points belonging to the environment
        outer_mask1 = np.where(radius_arm < d)
        outer_mask2 = np.where(radius_forearm < d2)
        outer_mask = np.intersect1d(outer_mask1, outer_mask2)

        # Publish
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera2_color_optical_frame'
        # Publish arm point cloud
        if np.size(arm_points) != 0:
            arm_pub = pc2.create_cloud_xyz32(header, arm_points)
            self.arm_pub.publish(arm_pub)
        # Publish environment point cloud
        if self.points[outer_mask,:].size != 0:
            env_pcd = pc2.create_cloud_xyz32(header, self.points[outer_mask,:])
            self.env_pub.publish(env_pcd)


    def get_joint_states(self, msg):
        self.rot_x = -msg.position[1]
        self.rot_y = -msg.position[2]
        self.rot_z = msg.position[0]
        self.elbow_angle = -msg.position[3]
        # self.get_logger().info(f'My log message {self.rot_x}')



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