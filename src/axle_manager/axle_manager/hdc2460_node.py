import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from axle_manager.hdc2460 import Hdc2460

facSerialPort = '/dev/ttyACM0'
bacSerialPort = '/dev/ttyACM1'

class Hdc2460Node(Node):
    def __init__(self):
        super().__init__("hdc2460")
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.fac = Hdc2460(facSerialPort)
        self.bac = Hdc2460(bacSerialPort)

        if self.fac.isConnected and self.bac.isConnected:
            self.get_logger().info("HDC2460s initialized.")
        else:
            self.get_logger().error("Unable to initialize Hdc2460s")
            #shutdown since we couldn't startup ports
            self.destroy_node()
            rclpy.shutdown()


    def callback(self, msg:Twist):
        left_speed = (msg.linear.x - msg.angular.z)
        right_speed = (msg.linear.x + msg.angular.z)

        # Send linear and angular velocities to serial
        self.fac.move(left_speed,right_speed)
        self.bac.move(left_speed,right_speed)
        self.get_logger().debug("Roboteq: Left={} Right={}".format(left_speed,right_speed))

    def destroy_node(self):
        super().destroy_node()
        self.fac.close
        self.bac.close
        self.get_logger().info("HDC2460s closed.")

def main(args=None):
    rclpy.init(args=args)

    node = Hdc2460Node()
    rclpy.spin(node)
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()