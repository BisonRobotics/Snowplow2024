from geometry_msgs.msg import Twist

from control_pkg.path_planning import turn_path
from control_pkg.commands import Command, SequentialCommandGroup
from control_pkg.wait_commands import WaitCommand
from control_pkg.turn_command import TurnToDegreesCommand
from typing import Callable
from time import time
from math import sqrt

class DriveTimeCommand(Command):
    """
    Command that drives the vehicle at the given speed for the given time
    """
    def __init__(self, speed: float, drive_time: float, drive: Callable[[float], None]):
        super().__init__()
        self.speed = float(speed)
        self.drive_time = drive_time
        self.drive = drive
        
    def initialize(self):
        self.end_time = time() + self.drive_time
        
    def execute(self):
        self.drive(self.speed)
        
    def is_finished(self) -> bool:
        return time() >= self.end_time
    
    def end(self):
        self.drive(float(0))

# measured in m/s
top_speed = 1
# measured in m/s/s
acceleration = 3
deceleration = 1
        
class DriveDistanceCommand(DriveTimeCommand):
    """
    Command that drives the vehicle the given distance at the given speed
    """
    def __init__(self, speed: float, distance: float, drive: Callable[[float], None]):
        super().__init__(speed=speed, drive_time=self.calculate_time(speed, distance), drive=drive)
        
    def calculate_time(self, speed, distance) -> float:
        real_speed = abs(speed) * top_speed
        
        acceleration_time = real_speed / acceleration
        deceleration_time = real_speed / deceleration
        
        if distance < real_speed * (acceleration_time + deceleration_time) / 2:
            return sqrt((2 * distance * acceleration * deceleration) / (acceleration + deceleration)) / acceleration
        
        hold_time = ((2 * distance / real_speed) - acceleration_time - deceleration_time) / 2
        return acceleration_time + hold_time

max_turn = 18.25

class DriveToWaypointCommand(SequentialCommandGroup):
    """
    Command that drives the vehicle to the given waypoint
    """
    def __init__(self, waypoint: Twist, get_position: Callable[[], Twist], get_pivot_position: Callable[[], float], drive_pivot: Callable[[int], None], drive: Callable[[float], None]):
        super().__init__()
        self.waypoint = waypoint
        self.get_position = get_position
        self.get_pivot_position = get_pivot_position
        self.drive_pivot = drive_pivot
        self.drive = drive
        
    def initialize(self):
        my_position = self.get_position()
        path = turn_path(start_point=(my_position.linear.x, my_position.linear.z), start_direction=my_position.angular.y, end_point=(self.waypoint.linear.x, self.waypoint.linear.z), end_direction=self.waypoint.angular.y)
    
        path_segment_1 = SequentialCommandGroup()
        path_segment_1.add_commands(
            TurnToDegreesCommand(path[0] * 18.624, self.get_pivot_position, self.drive_pivot),
            WaitCommand(2),
            DriveDistanceCommand(top_speed * path[1], path[2], self.drive)
        )
        
        path_segment_2 = SequentialCommandGroup()
        path_segment_2.add_commands(
            TurnToDegreesCommand(path[3] * 18.624, self.get_pivot_position, self.drive_pivot),
            WaitCommand(2),
            DriveDistanceCommand(top_speed * path[4], path[5], self.drive)
        )
        
        path_segment_3 = SequentialCommandGroup()
        path_segment_3.add_commands(
            TurnToDegreesCommand(path[6] * 18.624, self.get_pivot_position, self.drive_pivot),
            WaitCommand(2),
            DriveDistanceCommand(top_speed * path[7], path[8], self.drive)
        )
        
        needed_segments: list[SequentialCommandGroup] = []
        if path[2] > 0.2:
            needed_segments.append(path_segment_1)
        if path[5] > 0.2:
            needed_segments.append(path_segment_2)
        if path[8] > 0.2:
            needed_segments.append(path_segment_3)
            
        for i in range(len(needed_segments)):
            if i != len(needed_segments) - 1:
                needed_segments[i].add_command(WaitCommand(2))
            self.add_command(needed_segments[i])
            
        if len(self.commands) > 0:
            self.commands[0].initialize()
