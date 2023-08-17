import serial
from utilities.utilities.tools import Tools

baudRate = 115200
min_speed=-3
max_speed=3

class Hdc2460():

    def __init__(self, serialPort:str):
        self.port = serial.Serial(serialPort, baudRate, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False)

    def  isConnected(self) -> bool:
        return self.port.is_open

    def sendCommand(self, cmd):
        """Sends the serial data to both the HDC2460"""
        self.port.write(cmd + "\r")

    def move(self, left_speed:float, right_speed:float):
        """Tells the HDC2460 to move at set speed"""

        #clamp speed, format, and send it of to the roboteq
        left_speed = Tools.clamp(left_speed, min_speed, max_speed)
        right_speed = Tools.clamp(right_speed, min_speed, max_speed)

        if right_speed < 0:        
            self.send_Command("!a{right_speed:.2f}")        
        else:       
            self.send_Command("!A{right_speed:.2f}") 
        
        if left_speed < 0:        
            self.send_Command("!b{right_speed:.2f}")        
        else:       
            self.send_Command("!B{right_speed:.2f}") 

    def close(self):
        self.port.close()

