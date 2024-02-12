import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from axle_manager.hdc2460 import Hdc2460
from utilities.tools import Tools

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
                ('accelRate',31860),
                ('brakeRate',10620),
                ('pivotDevice',"FAC"),
                ('pivotExtendChannel',1),
                ('pivotRetractChannel',2),
                ('pivotSensorChannel',1),
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
        self.pivotpublisher = self.create_publisher(Float32, '/sensor/pivot', 10)
        #self.flEnc = self.create_publisher(Int32, '/sensor/wheel/frontleft', 10)
        #self.frEnc = self.create_publisher(Int32, '/sensor/wheel/frontright', 10)
        #self.rlEnc = self.create_publisher(Int32, '/sensor/wheel/rearleft', 10)
        #self.rrEnc = self.create_publisher(Int32, '/sensor/wheel/rearright', 10)
        
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
        self.pivotSensor = int(self.get_parameter('pivotSensorChannel').value)

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
       
        self.reading_from_sensors = False
        self.turn_angle = 0
        self.timer = self.create_timer(0.1,self.sensorsPub)

    def speed(self, msg:Twist):
        speed = Tools.clamp(msg.linear.x, -1, 1)
        
        percentage = 0.01 * self.turn_angle + 1
        offset = (speed * percentage) - speed
        
        left_speed = (speed + offset)
        right_speed = (speed - offset)
        # Send linear and angular velocities to serial
        self.fac.move(left_speed,right_speed)
        self.bac.move(left_speed,right_speed)
        self.get_logger().debug("Roboteq: Left={0:.2f} Right={0:.2f}".format(left_speed,right_speed))

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
        cmd = Vector3()
        cmd.x = msg.angular.x
        cmd.y = msg.angular.y
        cmd.z = msg.angular.z
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
        self.get_logger().debug("Roboteq: Val={0:.4f} Ch1={0} Ch2={0}".format(cmd.x,chR,chL))
        self.get_logger().debug("Roboteq: Val={0:.4f} Ch1={0} Ch2={0}".format(cmd.y,chU,chD))

    def publish_sensor_data(self):
        self.reading_from_sensors = True
        pivot = Float32()
        if self.pivotDevice.casefold() == "FAC".casefold():
            pivot.data = self.fac.readAnalogInput(self.pivotSensor)
        elif self.pivotDevice.casefold() == "BAC".casefold():
            pivot.data = self.bac.readAnalogInput(self.pivotSensor)
        else:
            self.get_logger().warning("No pivot device configured.")
        self.get_logger().debug("Roboteq: AI={0:.2f} Ch={1}".format(pivot.data,self.pivotSensor))
        self.pivotpublisher.publish(pivot)
        self.turn_angle = Tools.potentiometer_to_degrees(pivot.data)
        self.reading_from_sensors = False

    def sensorsPub(self):
        if not self.reading_from_sensors:
            t = threading.Thread(target=self.publish_sensor_data)
            t.start()

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
