import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int8
from axle_manager.hdc2460 import Hdc2460

class Hdc2460Node(Node):
    def __init__(self):
        super().__init__("hdc2460")
        self.set_parameters_atomically(self.declare_parameters(
            namespace='',
            parameters=[
                ('facSerialPort',"/dev/ttyS1"),
                ('bacSerialPort',"/dev/ttyS0"),
                ('serialBitRate',115200),
                ('leftChannel',1),
                ('rightChannel',2),
                ('maxSpeed',500),
                ('accelRate',5000),
                ('brakeRate',5000),
                ('pivotDevice',"FAC"),
                ('pivotExtendChannel',1),
                ('pivotRetractChannel',2),
                ('plowDevice',"FAC"),
                ('plowLeftChannel',3),
                ('plowRightChannel',4),
                ('plowUpChannel',5),
                ('plowDownChannel',6)
            ]
        ))
        #Start subscriptions
        self.speedSubscription = self.create_subscription(Twist, '/cmd_vel', self.speed, 10)
        self.speedSubscription  # prevent unused variable warning
        self.pivotSubscription = self.create_subscription(Int8, '/vehicle/pivot', self.pivot, 10)
        self.pivotSubscription  # prevent unused variable warning
        self.plowSubscription = self.create_subscription(Twist, '/vehicle/plow', self.plow, 10)
        self.plowSubscription  # prevent unused variable warning
        
        #Get all parameters
        facSerialPort = str(self.get_parameter('facSerialPort').value)
        bacSerialPort = str(self.get_parameter('bacSerialPort').value)
        bitRate = int(self.get_parameter('serialBitRate').value)
        leftChannel = int(self.get_parameter('leftChannel').value)
        rightChannel = int(self.get_parameter('rightChannel').value)
        maxSpeed = int(self.get_parameter('maxSpeed').value)
        accelRate = int(self.get_parameter('accelRate').value)
        brakeRate = int(self.get_parameter('brakeRate').value)
        self.pivotDevice = str(self.get_parameter('pivotDevice').value)
        self.pivotLeft = int(self.get_parameter('pivotExtendChannel').value)
        self.pivotRight = int(self.get_parameter('pivotRetractChannel').value)
        self.plowDevice = str(self.get_parameter('plowDevice').value)
        self.plowLeft = int(self.get_parameter('plowLeftChannel').value)
        self.plowRight = int(self.get_parameter('plowRightChannel').value)
        self.plowUp = int(self.get_parameter('plowUpChannel').value)
        self.plowDown = int(self.get_parameter('plowDownChannel').value)

        self.fac = Hdc2460(facSerialPort,bitRate,leftChannel,rightChannel)
        self.bac = Hdc2460(bacSerialPort,bitRate,leftChannel,rightChannel)

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