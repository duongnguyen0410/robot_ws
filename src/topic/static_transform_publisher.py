import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from std_msgs.msg import Float64

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribe to the topic that publishes theta
        self.subscription = self.create_subscription(
            Float64,
            'theta',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.theta = 0.0  # Initialize theta to 0

    def listener_callback(self, msg):
        self.theta = msg.data
        self.publish_static_transform()

    def publish_static_transform(self):
        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'chassis'
        static_transform_stamped.child_frame_id = 'caster_wheel'

        static_transform_stamped.transform.translation.x = 0.71
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = -0.035
        
        # Convert Euler angles to quaternion
        x = 1.57079633
        y = self.theta
        z = 0
        
        qx, qy, qz, qw = self.euler_to_quaternion(x, z, y)
        static_transform_stamped.transform.rotation.x = qx
        static_transform_stamped.transform.rotation.y = qy
        static_transform_stamped.transform.rotation.z = qz
        static_transform_stamped.transform.rotation.w = qw

        # Broadcast the transform
        self.static_broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info('Static transform published from chassis to caster_wheel')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        """
        import math
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
