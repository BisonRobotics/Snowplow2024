import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

class FQR(Node):
    
    def __init__(self):
        super().__init__('fqr')
        
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_timer(.01, self.publish_speed)
        
    def publish_speed(self):
        msg = Twist()
        msg.linear.x = 1.0
        self.speed_publisher.publish(msg)
        
def main():
    rclpy.init()
    
    node = FQR()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()