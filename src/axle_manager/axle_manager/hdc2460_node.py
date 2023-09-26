import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from axle_manager.hdc2460 import Hdc2460

class Hdc2460Node(Node):
    def __init__(self):
        super().__init__("hdc2460")
        self.declare_parameters(
            parameters=[
                ('facSerialPort',rclpy.Parameter.Type.STRING),
                ('bacSerialPort',rclpy.Parameter.Type.STRING),
                ('serialBitRate',rclpy.Parameter.Type.INTEGER),
                ('leftChannel',1,rclpy.Parameter.Type.INTEGER),
                ('rightChannel',2,rclpy.Parameter.Type.INTEGER),
                ('maxSpeed',1000,rclpy.Parameter.Type.INTEGER),
                ('accelRate',500,rclpy.Parameter.Type.INTEGER),
                ('brakeRate',500,rclpy.Parameter.Type.INTEGER)
            ]
        )

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.subscription  # prevent unused variable warning
        
        facSerialPort = self.get_parameter('facSerialPort')
        bacSerialPort = self.get_parameter('bacSerialPort')
        bitRate = self.get_parameter('serialBitRate')
        leftChannel = self.get_parameter('leftChannel')
        rightChannel = self.get_parameter('rightChannel')
        maxSpeed = self.get_parameter('maxSpeed')
        accelRate = self.get_parameter('accelRate')
        brakeRate = self.get_parameter('brakeRate')

        self.fac = Hdc2460(facSerialPort,bitRate,maxSpeed,leftChannel,rightChannel)
        self.bac = Hdc2460(bacSerialPort,bitRate,maxSpeed,leftChannel,rightChannel)

        if self.fac.isConnected and self.bac.isConnected:
            self.get_logger().info("HDC2460s connected.")
            self.fac.configure(maxSpeed,accelRate,brakeRate)
            self.get_logger().debug("FAC configured.")
            self.bac.configure(maxSpeed,accelRate,brakeRate)
            self.get_logger().debug("BAC configured.")
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