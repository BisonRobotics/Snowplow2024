import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int8
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
                ('brakeRate',500,rclpy.Parameter.Type.INTEGER),
                ('pivotDevice',rclpy.Parameter.Type.STRING),
                ('plowDevice',rclpy.Parameter.Type.STRING)
            ]
        )
        #Start subscriptions
        self.speedSubscription = self.create_subscription(Twist, '/cmd_vel', self.speed, 10)
        self.speedSubscription  # prevent unused variable warning
        self.pivotSubscription = self.create_subscription(Int8, '/vehicle/pivot', self.pivot, 10)
        self.pivotSubscription  # prevent unused variable warning
        self.plowSubscription = self.create_subscription(Twist, '/vehicle/plow', self.plow, 10)
        self.plowSubscription  # prevent unused variable warning
        
        #Get all parameters
        facSerialPort = str(self.get_parameter('facSerialPort'))
        bacSerialPort = str(self.get_parameter('bacSerialPort'))
        bitRate = int(self.get_parameter('serialBitRate'))
        leftChannel = int(self.get_parameter('leftChannel'))
        rightChannel = int(self.get_parameter('rightChannel'))
        maxSpeed = int(self.get_parameter('maxSpeed'))
        accelRate = int(self.get_parameter('accelRate'))
        brakeRate = int(self.get_parameter('brakeRate'))
        self.pivotDevice = str(self.get_parameter('pivotDevice'))
        self.pivotLeft = int(self.get_parameter('pivotExtendChannel'))
        self.pivotRight = int(self.get_parameter('pivotRetractChannel'))
        self.plowDevice = str(self.get_parameter('plowDevice'))
        self.plowLeft = int(self.get_parameter('plowLeftChannel'))
        self.plowRight = int(self.get_parameter('plowRightChannel'))
        self.plowUp = int(self.get_parameter('plowUpChannel'))
        self.plowDown = int(self.get_parameter('plowDownChannel'))

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


    def speed(self, msg:Twist):
        left_speed = (msg.linear.x - msg.angular.z)
        right_speed = (msg.linear.x + msg.angular.z)

        # Send linear and angular velocities to serial
        self.fac.move(left_speed,right_speed)
        self.bac.move(left_speed,right_speed)
        self.get_logger().debug("Roboteq: Left={} Right={}".format(left_speed,right_speed))

    def pivot(self, msg:Int8):
        cmd = int(msg.data)
        chL = int(self.pivotLeft)
        chR = int(self.pivotRight)
        if self.pivotDevice.casefold() == "FAC".casefold():
            self.fac.actuate(cmd,chL,chR)
        elif self.pivotDevice.casefold() == "BAC".casefold():
            self.bac.actuate(cmd,chL,chR)
        else:
            self.get_logger().warning("No pivot device configured.")
        self.get_logger().debug("Roboteq: Val={} Ch1={} Ch2={}".format(cmd,chL,chR))
        

    def plow(self, msg:Twist):
        cmd = Vector3(msg.angular)
        chL = int(self.plowLeft)
        chR = int(self.plowRight)
        chU = int(self.plowLeft)
        chD = int(self.plowLeft)
        if self.pivotDevice.casefold() == "FAC".casefold():
            self.fac.actuate(cmd.x,chR,chL)
            self.fac.actuate(cmd.y,chU,chD)
        elif self.pivotDevice.casefold() == "BAC".casefold():
            self.bac.actuate(cmd.x,chR,chL)
            self.bac.actuate(cmd.y,chU,chD)
        else:
            self.get_logger().warning("No plow device configured.")
        self.get_logger().debug("Roboteq: Val={} Ch1={} Ch2={}".format(cmd.x,chR,chL))
        self.get_logger().debug("Roboteq: Val={} Ch1={} Ch2={}".format(cmd.y,chU,chD))
        


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