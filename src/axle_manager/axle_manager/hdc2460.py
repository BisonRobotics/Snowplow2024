import serial
from utilities.utilities.tools import Tools

class Hdc2460():

    def __init__(self, serialPort:str,bitRate:int,maxSpeed:int):
        self.port = serial.Serial(serialPort, bitRate, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False)
        self.maxSpeed = maxSpeed

    def  isConnected(self) -> bool:
        return self.port.is_open

    def sendCommand(self, cmd):
        """Sends the serial data to both the HDC2460"""
        self.port.write(cmd + "\r")

    def move(self, left_speed:float, right_speed:float):
        """Tells the HDC2460 to move at set speed"""
        left_speed = round(left_speed*1000)
        right_speed = round(right_speed*1000)

        #clamp speed, format, and send it of to the roboteq
        left_speed = Tools.clamp(left_speed, -self.maxSpeed, self.maxSpeed)
        right_speed = Tools.clamp(right_speed, -self.maxSpeed, self.maxSpeed)

        self.send_Command("!G 2 {left_speed}") 
        self.send_Command("!G 1 {right_speed}")  

    def resetDigitalOut(self, bit:int):
        """"Will turn off digital output selected by the bit number."""
        self.send_Command("!D0 {bit}")  


    def close(self):
        self.port.close()

