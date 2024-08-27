#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import math

class ComputedVelSubscriber(Node):

    def __init__(self):
        super().__init__('computed_vel_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'computed_vel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float64, 'theta', 10)
        self.subscription  # prevent unused variable warning

        # Define the distances
        self.d = 0.48  # Distance between the left and right wheels (track width)
        self.L = 0.665  # Distance from wheel to caster wheel

    def listener_callback(self, msg):
        velocity_left = msg.data[0]
        velocity_right = msg.data[1]
        self.get_logger().info(f'Received velocities - Left: {velocity_left}, Right: {velocity_right}')

        # Calculate v and omega
        v = (velocity_left + velocity_right) / 2.0
        omega = (velocity_right - velocity_left) / self.d

        # Calculate R
        if omega != 0:
            R = v / omega
        else:
            R = float('inf')  # Robot is moving straight

        # Calculate the steering angle theta
        if v != 0 and R != float('inf'):
            theta = math.atan(self.L / R)
        elif v == 0 and omega != 0:
            # Robot is rotating in place, calculate theta based on omega
            if velocity_left < 0:
                theta = math.atan(self.d / 2.0 / self.L)
            else:
                theta = -math.atan(self.d / 2.0 / self.L)
        else:
            theta = 0.0  # No steering needed when moving straight or if R is 0

        self.get_logger().info(f'Computed values - v: {v}, omega: {omega}, R: {R}, theta: {theta}')

        # Publish the theta value
        theta_msg = Float64()
        theta_msg.data = theta
        self.publisher_.publish(theta_msg)
        self.get_logger().info(f'Published theta: {theta}')

def main(args=None):
    rclpy.init(args=args)
    node = ComputedVelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
