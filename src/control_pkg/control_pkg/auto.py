import rclpy

from rclpy.node import Node
from control_pkg.commands import Runner, Command

class Auto(Node):
    def __init__(self):
        super().__init__('auto')
        
        self.runner = Runner()
        self.create_timer(0.01, self.runner.run)
        
        self.runner.start_command(self.get_auto_command())
        
    def get_auto_command(self) -> Command:
        return Command()
        
def main():
    rclpy.init()
    
    node = Auto()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()