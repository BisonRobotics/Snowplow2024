import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist

from control_pkg.commands import Runner, Command
from control_pkg.drive_commands import DriveDistanceCommand, DriveToWaypointCommand
from control_pkg.control_pkg.wait_commands import WaitCommand, WaitUntilCommand
from control_pkg.control_pkg.turn_command import TurnToDegreesCommand

class Auto(Node):
    def __init__(self):
        super().__init__('auto')
        
        self.runner = Runner()
        self.create_timer(0.01, self.runner.run)
        
        self.pivot_position = None
        self.position = None
        
        self.pivot_position_updated = False
        self.position_updated = False
        
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Twist, '/apriltag', self.update_position, 10)
        self.create_subscription(Float32, '/sensor/pivot', self.update_pivot_position, 10)
        
        self.runner.start_command(self.get_auto_command())
        
    def get_auto_command(self) -> Command:
        left_waypoint = Twist()
        left_waypoint.linear.x = -2.5
        left_waypoint.linear.z = 2
        left_waypoint.angular.y = 180
        
        right_waypoint = Twist()
        right_waypoint.linear.x = 2.5
        right_waypoint.linear.z = 2
        right_waypoint.angular.y = 0
        
        return WaitUntilCommand(lambda : self.position_updated and self.pivot_position_updated)\
                .and_then(TurnToDegreesCommand(0, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveToWaypointCommand(left_waypoint, self.get_position, self.get_pivot_position, self.drive_pivot, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(0, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(1, 2, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(-22, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(1, 3.14 * 1.90475 / 4, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(-1, 3.14 * 1.90471 / 4, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(0, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(-1, 1.5, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(-18.624, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(-1, 2.25 * 3.14 / 2, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(0, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(-1, 1.5, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(DriveToWaypointCommand(right_waypoint, self.get_position, self.get_pivot_position, self.drive_pivot, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(0, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(1, 2, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(-22, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(1, 3.14 * 1.90475 / 4, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(-1, 3.14 * 1.90475 / 4, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(0, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(-1, 1.5, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(18.624, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(-1, 2.25 * 3.14 / 2, self.drive))\
                .and_then(WaitCommand(2))\
                .and_then(TurnToDegreesCommand(0, self.get_pivot_position, self.drive_pivot))\
                .and_then(WaitCommand(2))\
                .and_then(DriveDistanceCommand(-1, 1.5, self.drive))
    
    def drive_pivot(self, speed):
        msg = Int8()
        msg.data = speed
        self.pivot_publisher.publish(msg)
        
    def update_pivot_position(self, msg: Float32):
        self.pivot_position_updated = True
        self.pivot_position = msg.data
        
    def get_pivot_position(self) -> float:
        return self.pivot_position
    
    def update_position(self, msg):
        self.position_updated = True
        self.position = msg
        
    def get_position(self):
        return self.position
        
    def drive(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.speed_publisher.publish(msg)
        
def main():
    rclpy.init()
    
    node = Auto()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
