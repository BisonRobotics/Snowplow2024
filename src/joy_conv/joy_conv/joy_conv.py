import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from numpy import float32
from utilities.tools import Tools

# XBOX Axis
XBOX_LEFT_TRIGGER = 5
XBOX_RIGHT_TRIGGER = 4
XBOX_DPAD_UPDOWN = 7
XBOX_DPAD_LEFTRIGHT = 6
XBOX_RIGHT_Y = 3

# XBOX Buttons
XBOX_LEFT_PALM = 23
XBOX_RIGHT_PALM = 19
XBOX_Y_BUTTON = 4


class JoyConv(Node):
    
    def __init__(self):
        super().__init__('joy_conv')
        
        self.pivot_to_middle = False
        self.current_pivot_pos = 0
        self.degree_deadband = 0.5
        
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)
        self.plow_publisher = self.create_publisher(Twist, '/vehicle/plow', 10)

        self.pivot_sub = self.create_subscription(Float32, '/sensor/pivot', self.update_pivot, 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def update_pivot(self, msg: Float32):
        self.current_pivot_pos = Tools.potentiometer_to_degrees(msg.data)

    def joy_callback(self, msg:Joy):
        self.pivot_publisher.publish(self.calculate_pivot(msg))
        self.speed_publisher.publish(self.calculate_speed(msg))
        self.plow_publisher.publish(self.calculate_plow(msg))

    def calculate_pivot(self, joy_msg:Joy) -> Int8:
        msg = Int8()
        right = int(joy_msg.buttons[XBOX_RIGHT_PALM])
        left = int(joy_msg.buttons[XBOX_LEFT_PALM])
        y_button = int(joy_msg.buttons[XBOX_Y_BUTTON])
        if y_button > 0:
            self.pivot_to_middle = True
        if right > 0 or left > 0:
            self.pivot_to_middle = False
        if self.pivot_to_middle:
            if self.in_deadband():
                msg.data = 0
                self.pivot_to_middle = False
            else:
                msg.data = 1 if self.current_pivot_pos < 0 else -1
        else:
            msg.data = right - left
        return msg
    
    def in_deadband(self):
        lower_deadband = -self.degree_deadband
        upper_deadband = self.degree_deadband
        return self.current_pivot_pos >= lower_deadband and self.current_pivot_pos <= upper_deadband

    def calculate_speed(self, joy_msg:Joy) -> Twist:
        msg = Twist()
        msg.linear.x = joy_msg.axes[XBOX_RIGHT_Y] * abs(joy_msg.axes[XBOX_RIGHT_Y])
        return msg

    def calculate_plow(self, joy_msg:Joy) -> Twist:
        msg = Twist()
        msg.angular.x = joy_msg.axes[XBOX_DPAD_LEFTRIGHT]
        msg.angular.y = joy_msg.axes[XBOX_DPAD_UPDOWN]
        return msg

def main():
    rclpy.init()

    node = JoyConv()
    rclpy.spin(node)
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
