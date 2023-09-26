import serial
from numpy import float32
from utilities.utilities.tools import Tools

class Hdc2460():

    def __init__(self, serialPort:str,bitRate:int,leftChannel:int,rightChannel:int):
        self.port = serial.Serial(serialPort, bitRate, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False)
        self.maxSpeed = 1000
        self.leftChannel = leftChannel
        self.rightChannel = rightChannel

    def close(self):
        self.port.close()

    def  isConnected(self) -> bool:
        return self.port.is_open

    def move(self, left_speed:float32, right_speed:float32):
        """Tells both motors to move at set speed. Assumes speed input from -1 to 1."""
        self.setMotor(self.leftChannel,left_speed)
        self.setMotor(self.rightChannel,right_speed)

    def configure(self, maxSpeed:int,accelRate:int, brakeRate:int):
        self.setAccelration(accelRate)
        self.setDeccelration(brakeRate)
        self.maxSpeed = maxSpeed


#region Roboteq Driver
    def sendCommand(self, cmd):
        """Sends the serial data to both the HDC2460"""
        self.port.write(cmd + "\r")

    def setMotor(self,ch:int,speed:float32):
        """Tells the HDC2460 to move at set speed"""
        speed = round(speed*1000)

        #clamp speed, format, and send it of to the roboteq
        speed = Tools.clamp(speed, -self.maxSpeed, self.maxSpeed)

        #Roboteq speed input is from -1000 to 1000. Just in case max speed > 1000.
        speed = Tools.clamp(speed, -1000, 1000)
        #send command to 
        self.sendCommand("!G {ch} {speed}")

    def stopMotor(self, ch:int):
        """Stops the motor for the specified channel."""
        self.sendCommand("!MS {ch}")

    def stopAllMotors(self):
        """Stops both motors on the controller."""
        self.stopMotor(self.leftChannel)
        self.stopMotor(self.rightChannel)

    def resetDigitalOut(self):
        """"Will turn off all digital outputs."""
        self.sendCommand("!DS 0")  

    def setDigitalOut(self,bit:int,val:int):
        """"Will set digital output selected by the bit number."""
        if val==0:
            self.sendCommand("!D0 {bit}")  
        else:
            self.sendCommand("!D1 {bit}")  

    def setDeccelration(self, rate:int):
        """Set the rate of speed change during decceleration for a motor channel."""
        rate = Tools.clamp(rate, 0, 500000)
        self.sendCommand("!DC {self.leftChannel} {rate}")  
        self.sendCommand("!DC {self.rightChannel} {rate}")  

    def setAccelration(self, rate:int):
        """Set the rate of speed change during acceleration for a motor channel."""
        rate = Tools.clamp(rate, 0, 500000)
        self.sendCommand("!AC {self.leftChannel} {rate}")  
        self.sendCommand("!AC {self.rightChannel} {rate}")  



#endregion