
import numpy as np
from matplotlib import pyplot as plt
import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node

class JointPlotter(Node):

    def __init__(self):

        super().__init__('plotter')
        self.counter = 0
        self.depth_sub = self.create_subscription(
            JointState,
            '/roboy/pinky/control/cardsflow_joint_states',
            self.plot,
            3)

    def plot(self, msg):
        if self.counter % 10 == 0:
            stamp = msg.header.stamp
            time = stamp.sec
            plt.plot(time, msg.position[0], '*', color='red')
            plt.plot(time, msg.position[1], '*', color='blue')
            plt.plot(time, msg.position[2], '*', color='green')
            plt.axis("equal")
            plt.draw()
            plt.pause(0.00000000001)

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)

    depth_subscriber = JointPlotter()
    plt.ion()
    plt.show()

    rclpy.spin(depth_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
