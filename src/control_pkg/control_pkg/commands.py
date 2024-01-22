class Command:
    def __init__(self):
        pass
    
    def initialize(self):
        pass
    
    def execute(self):
        pass
    
    def is_finished(self) -> bool:
        return False
    
    def end(self):
        pass
    
    def along_with(self, other_command: 'Command') -> 'Command':
        group: ParallelCommandGroup = ParallelCommandGroup()
        group.add_commands(self, other_command)
        return group
    
    def and_then(self, next_command: 'Command') -> 'Command':
        group: SequentialCommandGroup = SequentialCommandGroup()
        group.add_commands(self, next_command)
        return group
    
class SequentialCommandGroup(Command):
    def __init__(self):
        super().__init__()
        self.commands: list[Command] = []
        
    def add_command(self, command: Command):
        self.commands.append(command)
        
    def add_commands(self, *commands: Command):
        for command in commands:
            self.add_command(command)
            
    def initialize(self):
        if len(self.commands) > 0:
            self.commands[0].initialize()
        
    def execute(self):
        current_command: Command | None = self.commands[0] if len(self.commands) > 0 else None
        while current_command is not None and current_command.is_finished():
            self.commands.remove(current_command)
            current_command = self.commands[0] if len(self.commands) > 0 else None
            current_command.initialize()
        if current_command is not None:
            current_command.execute()

    def is_finished(self) -> bool:
        return len(self.commands) == 0
    
    def end(self):
        if self.commands[0] is not None:
            self.commands[0].end()

class ParallelCommandGroup(Command):
    def __init__(self):
        super().__init__()
        self.commands: list[Command] = []
        
    def add_command(self, command: Command):
        self.commands.append(command)
        
    def add_commands(self, *commands: Command):
        for command in commands:
            self.add_command(command)
            
    def initialize(self):
        for command in self.commands:
            command.initialize()
            
    def execute(self):
        for command in self.commands:
            if command.is_finished():
                command.end()
                self.commands.remove(command)
                continue
            command.execute()
            
    def is_finished(self) -> bool:
        return all(map(lambda command : command.is_finished(), self.commands))
    
    def end(self):
        for command in self.commands:
            command.end()
    
class Runner:
    def __init__(self):
        self.commands: list[Command] = []
        
    def start(self, command: Command):
        self.commands.append(command)
        command.initialize()
        
    def run(self):
        for command in self.commands:
            if command.is_finished():
                command.end()
                self.commands.remove(command)
                continue
            command.execute()