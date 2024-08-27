import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

rclpy.init()
node = rclpy.create_node('position_velocity_publisher')
pub = node.create_publisher(JointState, 'joint_states', 10)

# Spin in a separate thread
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()

joint_state_position = JointState()
joint_state_position.name = ["caster_wheel_joint"]
joint_state_position.position = [0.0]

rate = node.create_rate(10)

try:
    while rclpy.ok():
        # Get current time
        now = node.get_clock().now().to_msg()

        # Set header time for position and velocity messages
        joint_state_position.header.stamp = now
        pub.publish(joint_state_position)
        rate.sleep()
except KeyboardInterrupt:
    pass

rclpy.shutdown()
thread.join()
