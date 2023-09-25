import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# XBOX Axis
XBOX_LEFT_TRIGGER = 5
XBOX_RIGHT_TRIGGER = 4
XBOX_DPAD_UPDOWN = 7
XBOX_DPAD_LEFTRIGHT = 6
XBOX_RIGHT_Y = 3

# XBOX Buttons
XBOX_LEFT_PALM = 23
XBOX_RIGHT_PALM = 19

def deadband(value, size):
    return value if abs(value) >= size else 0

class JoyConv(Node):
    
    def __init__(self):
        super().__init__('joy_conv')
        
        self.turn_publisher = self.create_publisher(Float64, '/drive/turn', 10)
        self.speed_publisher = self.create_publisher(Float64, '/drive/speed', 10)
        self.pitch_publisher = self.create_publisher(Float64, '/plow/pitch', 10)
        self.yaw_publisher = self.create_publisher(Float64, '/plow/yaw', 10)
        self.roll_publisher = self.create_publisher(Float64, '/plow/roll', 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, joy_msg):
        self.turn_publisher.publish(self.calculate_turn(joy_msg))
        self.speed_publisher.publish(self.calculate_speed(joy_msg))
        self.pitch_publisher.publish(self.calculate_pitch(joy_msg))
        self.yaw_publisher.publish(self.calculate_yaw(joy_msg))
        self.roll_publisher.publish(self.calculate_roll(joy_msg))

    def calculate_turn(self, joy_msg):
        msg = Float64()
        msg.data = float(msg.buttons[XBOX_RIGHT_PALM] - msg.buttons[XBOX_LEFT_PALM])
        return msg

    def calculate_speed(self, joy_msg):
        msg = Float64()
        msg.data = -deadband(joy_msg.axes[XBOX_RIGHT_Y], 0.005) # Deadband set to .5% may not be needed, but probably safer
        return msg

    def calculate_pitch(self, joy_msg):
        msg = Float()
        msg.data = joy_msg.axes[XBOX_DPAD_UPDOWN]
        return msg

    def calculate_yaw(self, joy_msg):
        msg = Float64()
        msg.data = joy_msg.axes[XBOX_DPAD_LEFTRIGHT]
        return msg

    def calculate_roll(self, joy_msg):
        msg = Float64()
        msg.data = deadband(joy_msg.axes[XBOX_RIGHT_TRIGGER] - joy_msg.axes[XBOX_LEFT_TRIGGER], 0.005) # Deadband set to .5% may not be needed, but probably safer
        return msg

def main():
    rclpy.init()
    joy_conv = JoyConv()
    rclpy.spin(joy_conv)
    joy_conv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()