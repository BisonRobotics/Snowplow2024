import rclpy

from rclpy.node import Node
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Twist
from utilities.tools import Tools
import time
import threading

class Auto(Node):
    
    def __init__(self):
        super().__init__('auto')
        
        self.running = False
        self.state = 0
        self.pivot_updated = False
        self.pivot_position = 0
        self.degree_deadband = 0.2
        
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 1000)
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)

        self.pivot_sub = self.create_subscription(Float32, '/sensor/pivot', self.update_pivot, 10)
    
        self.create_timer(0.01, self.determine_control)
        
    def determine_control(self):
        if not self.pivot_updated:
            return
        match self.state:
            case 0:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 1:
                self.get_logger().info('driving at 100% speed for 2 seconds')
                self.drive_speed_time(1.0, 1)
                return
            case 2:
                self.get_logger().info('turning to -20.75 degrees')
                self.turn_to_degrees(-20.75)
                return
            case 3:
                self.get_logger().info('driving at 100% speed for 2 seconds')
                self.drive_speed_time(1.0, 3.14)
                return
            case 4:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 5:
                self.get_logger().info('driving at 100% speed for 2 seconds')
                self.drive_speed_time(1.0, 1.5)
                return
            case 6:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 7:
                self.get_logger().info('driving at -100% speed for 2 seconds')
                self.drive_speed_time(-1.0, 1.5)
                return
            case 8:
                self.get_logger().info('turning to -20.75 degrees')
                self.turn_to_degrees(-20.75)
                return
            case 9:
                self.get_logger().info('driving at -100% speed for 2 seconds')
                self.drive_speed_time(-1.0, 3.14)
                return
            case 10:
                self.get_logger().info('turning to 20.75 degrees')
                self.turn_to_degrees(20.75)
                return
            case 11:
                self.get_logger().info('driving at 100% speed for 2 seconds')
                self.drive_speed_time(1.0, 3.14)
                return
            case 12:
                self.get_logger().info('turing to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 13:
                self.get_logger().info('driving at 100% speed for 2 seconds')
                self.drive_speed_time(1.0, 1.5)
                return
            case 14:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 15:
                self.get_logger().info('driving at -100% speed for 2 seconds')
                self.drive_speed_time(-1.0, 1.5)
                return
            case 16:
                self.get_logger().info('turning to 20.75 degrees')
                self.turn_to_degrees(20.75)
                return
            case 17:
                self.get_logger().info('driving at -100% speed for 2 seconds')
                self.drive_speed_time(-1.0, 3.14)
                return
            case 18:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 19:
                self.get_logger().info('driving at -100% speed for 2 seconds')
                self.drive_speed_time(-1.0, 1)
                return
            case _:
                self.get_logger().info('done')
                return

    def turn_to_degrees(self, degrees):
        msg = Int8()
        if abs(self.pivot_position - degrees) <= self.degree_deadband:
            self.state += 1
            msg.data = 0
        else:
            msg.data = 1 if self.pivot_position < degrees else -1
        self.pivot_publisher.publish(msg)

    def drive_speed_time(self, speed, run_time):
        if not self.running:
            self.end_time = time.time() + run_time
            self.running = True
        msg = Twist()
        if time.time() >= self.end_time:
            msg.linear.x = 0.0
            self.state += 1
            self.running = False
        else:
            msg.linear.x = speed
        self.get_logger().info(f'publishing speed: {msg.linear.x}')
        self.speed_publisher.publish(msg)
        self.get_logger().info(f'published speed: {msg.linear.x}')

    def wait_time(self, run_time):
        if not self.running:
            self.end_time = time.time() + run_time
            self.running = True
        if time.time() >= self.end_time:
            self.state += 1
            self.running = False

    def update_pivot(self, msg: Float32):
        self.pivot_updated = True
        self.pivot_position = Tools.potentiometer_to_degrees(msg.data)
        self.get_logger().info(f'updated pivot positon to {self.pivot_position}')
        
def main():
    rclpy.init()
    
    node = Auto()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
