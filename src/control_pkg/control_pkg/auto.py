import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist

from control_pkg.commands import Runner, Command
from control_pkg.drive_commands import DriveDistanceCommand, DriveToWaypointCommand
from control_pkg.wait_commands import WaitCommand, WaitUntilCommand
from control_pkg.turn_command import TurnToDegreesCommand

class Auto(Node):
    def __init__(self):
        super().__init__('auto')
        
        # Set up ROS stuff
        self.pivot_position = None
        self.position = None
        
        self.pivot_position_updated = False
        self.position_updated = False
        
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Twist, '/apriltag', self.update_position, 10)
        self.create_subscription(Float32, '/sensor/pivot', self.update_pivot_position, 10)
        
        # Start runner
        self.runner = Runner()
        self.create_timer(0.01, self.runner.run)
        
        # Start auto command
        self.runner.start_command(self.get_auto_command())
        
    def get_auto_command(self) -> Command:
        """
        Generates and returns the command to start when auto is started

        Returns:
            Command: Command to start when auto is started
        """
        # Waypoint after left turn out
        left_waypoint = Twist()
        left_waypoint.linear.x = -2.5
        left_waypoint.linear.z = 2
        left_waypoint.angular.y = 180
        
        # Waypoint after right turn out
        right_waypoint = Twist()
        right_waypoint.linear.x = 2.5
        right_waypoint.linear.z = 2
        right_waypoint.angular.y = 0
        
        # Factory functions for removing redundancy
        wait = lambda time_seconds : WaitCommand(time_seconds)
        turn_to_degrees = lambda degrees : TurnToDegreesCommand(degrees, self.get_pivot_position, self.drive_pivot)
        drive_to_waypoint = lambda waypoint : DriveToWaypointCommand(waypoint, self.get_position, self.get_pivot_position, self.drive_pivot, self.drive)
        drive_distance = lambda speed, distance : DriveDistanceCommand(speed, distance, self.drive)
        wait_until = lambda condition : WaitUntilCommand(condition)

        # Creating and returning command
        return wait_until(lambda : self.position_updated and self.pivot_position_updated)\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_to_waypoint(left_waypoint))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-22))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 3.14 * 1.90471 / 4))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-18.624))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 2.25 * 3.14 / 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))\
                .and_then(wait(2))\
                .and_then(drive_to_waypoint(right_waypoint))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-22))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(18.624))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 2.25 * 3.14 / 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))
    
    def drive_pivot(self, speed) -> None:
        """
        Drives the pivot left or right or stops the pivot

        Args:
            speed (int): -1 for left, 1 for right, 0 for stop
        """
        msg = Int8()
        msg.data = speed
        self.pivot_publisher.publish(msg)
        
    def update_pivot_position(self, msg: Float32) -> None:
        """
        Updates the pivot position when a new message is heard

        Args:
            msg (Float32): Message containing the new pivot position
        """
        self.pivot_position_updated = True
        self.pivot_position = msg.data
        
    def get_pivot_position(self) -> float:
        """
        Gets the current pivot position

        Returns:
            float: Current pivot position
        """
        return self.pivot_position
    
    def update_position(self, msg: Twist) -> None:
        """
        Updates the position of the vehicle when a new message is heard

        Args:
            msg (Twist): Message containing the new position
        """
        self.position_updated = True
        self.position = msg
        
    def get_position(self) -> float:
        """
        Gets the current position of the vehicle

        Returns:
            float: Current position of the vehicle
        """
        return self.position
        
    def drive(self, speed: float) -> None:
        """
        Drives the vehicle at the given speed

        Args:
            speed (float): Speed to drive at in range [-1..1]
        """
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
