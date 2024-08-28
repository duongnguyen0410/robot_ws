import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 

class IntPublisherNode(Node):
    def __init__(self):
        super().__init__('int_publisher_node')
        self.publisher_ = self.create_publisher(Int32, 'int_topic', 10)  
        self.timer = self.create_timer(0.5, self.publish_data)  

    def publish_data(self):
        msg = Int32()
        msg.data = 1
        self.publisher_.publish(msg) 

def main(args=None):
    rclpy.init(args=args)
    node = IntPublisherNode()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
