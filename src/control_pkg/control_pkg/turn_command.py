from control_pkg.commands import Command
from typing import Callable
from utilities.tools import Tools

degree_deadband = 0.2

class TurnToDegreesCommand(Command):
    """
    Command that turns the pivot to the given degrees
    """
    def __init__(self, degrees: float, get_pivot_degrees: Callable[[], float], drive_pivot: Callable[[int], None]):
        super().__init__()
        self.target_degrees = degrees
        self.get_pivot_degrees = lambda : Tools.potentiometer_to_degrees(get_pivot_degrees())
        self.drive_pivot = drive_pivot
        
    def in_deadband(self) -> bool:
        return abs(self.get_pivot_degrees() - self.target_degrees) <= degree_deadband
        
    def execute(self):
        if self.in_deadband():
            self.drive_pivot(0)
        else:
            self.drive_pivot(-1 if self.get_pivot_degrees() > self.target_degrees else 1)
            
    def is_finished(self) -> bool:
        return self.in_deadband()
    
    def end(self):
        self.drive_pivot(0)
