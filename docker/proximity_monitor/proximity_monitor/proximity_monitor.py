import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
# from proximity_monitor.msg import ProximityWarning

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

# ProximityWarning message
# Float32MultiArray: [warning, proximity, threshold]
# warning: 1 if the distance is less than the threshold
# proximity: distance to the nearest point (meters)
# threshold: distance threshold (meters)

class ProximityMonitorNode(Node):
    def __init__(self):
        super().__init__('proximity_monitor')

        # Declare parameters and initialize them
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('proximity_warning_threshold', 0.2)

        # Get parameters
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.threshold = self.get_parameter('proximity_warning_threshold').get_parameter_value().double_value

        # Publishers and Subscribers
        self.warning_topic = f'/roboy/pinky/sensing/{self.camera_name}/proximity_warning'

        self.warning_publisher = self.create_publisher(Float32MultiArray, self.warning_topic, 10)
        # self.second_warning_publisher = self.create_publisher(Float32MultiArray, self.second_warning_topic, 10)

        self.camera_subscriber = self.create_subscription(Image, f'/{self.camera_name}/depth/image_rect_raw', self.camera_callback, 10)

        self.get_logger().info(f'Listening for the camera on /{self.camera_name}/depth/image_rect_raw')

        self.last_warning = 0.0
        self.last_proximity = None

        self.timer = self.create_timer(0.025, self.publish_warning)  # 40 Hz

        # Dynamic reconfiguration callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    def process_camera_data(self, data):
        proximity = self.nearest_point_distance(data)
        # relative_proximity = proximity / self.threshold
        warning = 1.0 if proximity < self.threshold else 0.0
        return warning, proximity

    def camera_callback(self, data):
        self.last_warning, self.last_proximity = self.process_camera_data(data)
        # self.get_logger().debug(f'Proximity: {self.last_proximity}, Warning: {self.last_warning}')
        # cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        # compressed_img = cv2.imencode('.jpg', cv_img)[1].tostring()
        # compressed_msg = CompressedImage()
        # compressed_msg.header = data.header
        # compressed_msg.format = "jpeg"
        # compressed_msg.data = compressed_img
        # self.get_logger().info(f'Publishing image')
        # self.image_publisher.publish(compressed_msg)

    def publish_warning(self):
        warning_msg = Float32MultiArray()

        if self.last_warning:
            self.get_logger().info(f'Publishing warning: {self.last_proximity}')
        else:
            self.get_logger().debug(f'Publishing NO warning')

        warning_msg.data = [
            self.last_warning,
            self.last_proximity or float('NaN'),
            self.threshold
        ]
        self.warning_publisher.publish(warning_msg)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'proximity_warning_threshold' and param.type_ == param.PARAMETER_DOUBLE:
                self.threshold = param.value
                self.get_logger().info(f'New proximity_warning_threshold set: {self.threshold}')
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=False)
    
    def nearest_point_distance(self, ros_image):
        try:
            depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
            depth_array = np.array(depth_image, dtype=np.float64)
            min_distance = float(np.min(depth_array[depth_array!=0]))
            return min_distance * 0.001  # Convert to meters
        except CvBridgeError as e:
            self.get_logger().error(e)

def main(args=None):
    rclpy.init(args=args)
    proximity_monitor_node = ProximityMonitorNode()
    proximity_monitor_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    proximity_monitor_node.get_logger().info('Proximity Monitor Node started')
    rclpy.spin(proximity_monitor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
