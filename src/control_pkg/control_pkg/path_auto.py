import rclpy

from rclpy.node import Node
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Twist
from control_pkg.path_planning import turn_path
from utilities.tools import Tools
import time

class PathAuto(Node):
    
    def __init__(self):
        super().__init__('path_auto')
        
        self.running = False
        self.state = 0
        self.sub_state = 0
        self.pivot_updated = False
        self.position = None
        self.pivot_position = 0
        self.degree_deadband = 0.2
        self.path = None
        
        self.left_waypoint = Twist()
        self.left_waypoint.linear.x = -2.5
        self.left_waypoint.linear.z = 2
        self.left_waypoint.angular.y = 180
        
        self.right_waypoint = Twist()
        self.right_waypoint.linear.x = 2.5
        self.right_waypoint.linear.z = 2
        self.right_waypoint.angular.y = 0
        
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)
        
        self.pivot_sub = self.create_subscription(Float32, '/sensor/pivot', self.update_pivot, 10)
        self.position_sub = self.create_subscription(Twist, '/apriltag', self.update_position, 10)
        
        self.create_timer(0.01, self.determine_control)
        
    def update_pivot(self, msg):
        self.pivot_updated = True
        self.pivot_position = Tools.potentiometer_to_degrees(msg.data)
        
    def update_position(self, msg):
        self.position = msg
        
    def determine_control(self):
        if not self.pivot_updated or self.position is None:
            return
        path = None
        match self.state:
            case 0:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 1:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 2:
                self.get_logger().info('generating path')
                self.path = turn_path(start_point=(self.position.linear.x, self.position.linear.z), start_direction=self.position.angular.y, end_point=(self.left_waypoint.linear.x, self.left_waypoint.linear.z), end_direction=self.left_waypoint.angular.y)
                self.state += 1
                return
            case 3:
                self.get_logger().info('running first segment of the path')
                if self.path[2] < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path[0] * 18.624)
                return
            case 4:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 5:
                self.get_logger().info('running first segment of the path')
                self.drive_distance(self.path[1] * 1.0, self.path[2])
                return
            case 6:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 7:
                self.get_logger().info('running second segment of the path')
                if self.path[5] < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path[3] * 18.624)
                return
            case 8:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 9:
                self.get_logger().info('running second segment of the path')
                self.drive_distance(self.path[4] * 1.0, self.path[5])
                return
            case 10:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 11:
                self.get_logger().info('running the third segment of the path')
                if self.path[8] < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path[6] * 18.624)
                return
            case 12:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 13:
                self.get_logger().info('running the third segment of the path')
                self.drive_distance(self.path[7] * 1.0, self.path[8])
                return
            case 14:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                self.path = None
                return
            case 15:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 16:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 17:
                self.get_logger().info('driving at 100% speed for 2 meters')
                self.drive_distance(1.0, 2)
                return
            case 18:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 19:
                self.get_logger().info('driving at -100% speed for 1.5 meters')
                self.drive_distance(-1.0, 1.5)
                return
            case 20:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 21:
                self.get_logger().info('turning to -18.624 degrees')
                self.turn_to_degrees(-18.624)
                return
            case 22:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 23:
                self.get_logger().info('driving at -100% speed for 3.14')
                self.drive_distance(-1.0, 2.25 * 3.14 / 2)
                return
            case 24:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 25:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 26:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 27:
                self.get_logger().info('driving at -100% speed for 2 meters')
                self.drive_distance(-1.0, 2)
                return
            case 28:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 29:
                self.get_logger().info('generating path')
                self.path = turn_path(start_point=(self.position.linear.x, self.position.linear.z), start_direction=self.position.angular.y, end_point=(self.right_waypoint.linear.x, self.right_waypoint.linear.z), end_direction=self.right_waypoint.angular.y)
                self.state += 1
                return
            case 30:
                self.get_logger().info('running first segment of the path')
                if self.path[2] < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path[0] * 18.624)
                return
            case 31:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 32:
                self.get_logger().info('running first segment of the path')
                self.drive_distance(self.path[1] * 1.0, self.path[2])
                return
            case 33:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 34:
                self.get_logger().info('running second segment of the path')
                if self.path[5] < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path[3] * 18.624)
                return
            case 35:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 36:
                self.get_logger().info('running second segment of the path')
                self.drive_distance(self.path[4] * 1.0, self.path[5])
                return
            case 37:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 38:
                self.get_logger().info('running the third segment of the path')
                if self.path[8] < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path[6] * 18.624)
                return
            case 39:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 40:
                self.get_logger().info('running the third segment of the path')
                self.drive_distance(self.path[7] * 1.0, self.path[8])
                return
            case 41:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                self.path = None
                return
            case 42:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 43:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 44:
                self.get_logger().info('driving at 100% speed for 2 meters')
                self.drive_distance(1.0, 2)
                return
            case 45:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 46:
                self.get_logger().info('driving at -100% speed for 2.5 meters')
                self.drive_distance(-1.0, 2.5)
                return
            case 47:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 48:
                self.get_logger().info('turning to 18.624 degrees')
                self.turn_to_degrees(18.624)
                return
            case 49:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 50:
                self.get_logger().info('driving at -100% speed for 3.14 meters')
                self.drive_distance(-1.0, 2.25 * 3.14 / 2)
                return
            case 51:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 52:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 53:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 54:
                self.get_logger().info('driving at -100% speed for 2 meters')
                self.drive_distance(-1.0, 2)
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
        
    def drive_distance(self, speed, distance):
        run_time = 0
        if distance >= 2/3:
            run_time = (1/3) + distance - (2/3)
        else:
            run_time = ((distance / (2/3)) ** 2) / 3
        self.drive_speed_time(speed, run_time)
    
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
        self.speed_publisher.publish(msg)

    def wait_time(self, run_time):
        if not self.running:
            self.end_time = time.time() + run_time
            self.running = True
        if time.time() >= self.end_time:
            self.state += 1
            self.running = False

def main():
    rclpy.init()
    
    node = PathAuto()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    