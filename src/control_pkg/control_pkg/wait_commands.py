from control_pkg.commands import Command
from time import time
from typing import Callable

class WaitCommand(Command):
    def __init__(self, wait_time: float):
        super().__init__()
        self.wait_time = wait_time
        
    def initialize(self):
        self.end_time = time() + self.wait_time
        
    def is_finished(self) -> bool:
        return time() >= self.end_time
    
class WaitUntilCommand(Command):
    def __init__(self, condition: Callable[[], bool]):
        super().__init__()
        self.condition = condition
        
    def is_finished(self) -> bool:
        return self.condition()
