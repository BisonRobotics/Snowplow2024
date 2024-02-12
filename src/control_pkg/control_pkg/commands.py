class Command:
    """
    The basic building block of the command-based framework
    """
    def __init__(self):
        pass
    
    def initialize(self) -> None:
        """
        Runs once when the command starts
        """
        pass
    
    def execute(self) -> None:
        """
        Runs periodically while the command is active
        """
        pass
    
    def is_finished(self) -> bool:
        """
        Returns whether or not the command is completed. Once this returns True, the command will be stopped

        Returns:
            bool: If the command is finished executing
        """
        return False
    
    def end(self) -> None:
        """
        Runs once when the command is finished executing
        """
        pass
    
    def along_with(self, other_command: 'Command') -> 'Command':
        """
        Runs another command at the same time as this command.
        The resulting command will be finished once this command and the other command
        are finished.

        Args:
            other_command (Command): Command to run at the same time

        Returns:
            Command: Resulting command group
        """
        group: ParallelCommandGroup = ParallelCommandGroup()
        group.add_commands(self, other_command)
        return group
    
    def and_then(self, next_command: 'Command') -> 'Command':
        """
        Runs another command after this command is finished. The resulting command
        will be finished once the next command is finished

        Args:
            next_command (Command): Command to run next

        Returns:
            Command: Resulting command group
        """
        group: SequentialCommandGroup = SequentialCommandGroup()
        group.add_commands(self, next_command)
        return group
    
class SequentialCommandGroup(Command):
    """
    Command group that runs commands one after another. It is finished once all sub-commands are finished
    """
    def __init__(self):
        super().__init__()
        self.commands: list[Command] = []
        
    def add_command(self, command: Command) -> None:
        self.commands.append(command)
        
    def add_commands(self, *commands: Command) -> None:
        for command in commands:
            self.add_command(command)
            
    def initialize(self) -> None:
        if len(self.commands) > 0:
            self.commands[0].initialize()
        
    def execute(self) -> None:
        current_command: Command | None = self.commands[0] if len(self.commands) > 0 else None
        while current_command is not None and current_command.is_finished():
            current_command.end()
            self.commands.remove(current_command)
            current_command = self.commands[0] if len(self.commands) > 0 else None
            if current_command is not None:
                current_command.initialize()
        if current_command is not None:
            current_command.execute()

    def is_finished(self) -> bool:
        return len(self.commands) == 0
    
    def end(self) -> None:
        if len(self.commands) > 0:
            self.commands[0].end()

class ParallelCommandGroup(Command):
    """
    Command group that runs commands one after another. It is finished once all sub-commands are finished
    """
    def __init__(self):
        super().__init__()
        self.commands: list[Command] = []
        
    def add_command(self, command: Command) -> None:
        self.commands.append(command)
        
    def add_commands(self, *commands: Command) -> None:
        for command in commands:
            self.add_command(command)
            
    def initialize(self) -> None:
        for command in self.commands:
            command.initialize()
            
    def execute(self) -> None:
        for command in self.commands:
            if command.is_finished():
                command.end()
                self.commands.remove(command)
                continue
            command.execute()
            
    def is_finished(self) -> bool:
        return all(map(lambda command : command.is_finished(), self.commands))
    
    def end(self) -> None:
        for command in self.commands:
            command.end()
    
class Runner:
    """
    Backbone of the command based framework. Manages scheduling and running of commands
    """
    def __init__(self):
        self.commands: list[Command] = []
        
    def start_command(self, command: Command) -> None:
        """
        Starts the execution of a command

        Args:
            command (Command): Command to start
        """
        self.commands.append(command)
        command.initialize()
        
    def run(self) -> None:
        """
        Runs all active commands and stops finished commands
        """
        for command in self.commands:
            if command.is_finished():
                command.end()
                self.commands.remove(command)
                continue
            command.execute()
