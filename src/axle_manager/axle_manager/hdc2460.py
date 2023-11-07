import serial
import re
import threading
from time import sleep
from numpy import float32
from utilities.tools import Tools

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
        left_speed = Tools.clamp(left_speed,-2,2)
        right_speed = Tools.clamp(right_speed,-2,2)
        
        self.setMotor(self.leftChannel,left_speed)
        self.setMotor(self.rightChannel,right_speed)

    def actuate(self, val:int, extendCh:int, retractCh:int):
        """Controls an actuator using 2 digital output channels"""
        val = Tools.clamp(val, -1, 1)
        if val > 0:
            self.setDigitalOut(extendCh,1)
            self.setDigitalOut(retractCh,0)
        elif val < 0:
            self.setDigitalOut(extendCh,0)
            self.setDigitalOut(retractCh,1)
        else:
            self.setDigitalOut(extendCh,0)
            self.setDigitalOut(retractCh,0)

    def configure(self, maxSpeed:int,accelRate:int, brakeRate:int):
        """Configures the HDC2460"""
        self.setAccelration(accelRate)
        self.setDeccelration(brakeRate)
        self.maxSpeed = maxSpeed


#region Roboteq Driver
    def sendCommand(self, cmd):
        """Sends the serial data to both the HDC2460"""
        self.port.write((cmd + "\r").encode())

    def setMotor(self,ch:int,speed:float32):
        """Tells the HDC2460 to move at set speed"""
        speed = round(speed*500)
        #clamp speed, format, and send it of to the roboteq
        #speed = Tools.clamp(speed, -self.maxSpeed, self.maxSpeed)

        #Roboteq speed input is from -1000 to 1000. Just in case max speed > 1000.
        speed = Tools.clamp(speed, -1000, 1000)
        #send command to
        self.sendCommand(f"!G {ch} {speed}")

    def stopMotor(self, ch:int):
        """Stops the motor for the specified channel."""
        self.sendCommand(f"!MS {ch}")

    def stopAllMotors(self):
        """Stops both motors on the controller."""
        self.stopMotor(self.leftChannel)
        self.stopMotor(self.rightChannel)

    def resetDigitalOut(self):
        """"Will turn off all digital outputs."""
        self.sendCommand(f"!DS 0")  

    def setDigitalOut(self,bit:int,val:int):
        """"Will set digital output selected by the bit number."""
        val = Tools.clamp(val, 0, 1)
        val = 1 if val == 1 else 0
        self.sendCommand(f"!D{val} {bit}")

    def setDeccelration(self, rate:int):
        """Set the rate of speed change during decceleration for a motor channel."""
        rate = Tools.clamp(rate, 0, 500000)
        self.sendCommand(f"!DC {self.leftChannel} {rate}")  
        self.sendCommand(f"!DC {self.rightChannel} {rate}")  

    def setAccelration(self, rate:int):
        """Set the rate of speed change during acceleration for a motor channel."""
        rate = Tools.clamp(rate, 0, 500000)
        self.sendCommand(f"!AC {self.leftChannel} {rate}")  
        self.sendCommand(f"!AC {self.rightChannel} {rate}")  

    def readAnalogInput(self,channel:int)->float:
        """Reads the raw analog value"""
        self.port.reset_input_buffer()
        self.sendCommand(f"?AI {channel}")
        line = ''
        while (not re.search(r'AI=-?\d+\D', line)):
            line += str(self.port.read(self.port.in_waiting))[2:-1]
        result = float(re.search(r'AI=-?\d+', line).group()[3:])
        return result
    
    def readRPM(self,channel:int)->int:
        """Reports the actual speed measured by the encoders as the actual RPM value. To report RPM accurately, the correct Pulses per Revolution (PPR) must be stored in the encoder configuration """
        self.port.reset_input_buffer()
        self.sendCommand("?S {channel}")
        line = ''
        while (not re.search(r'AI=-?\d+\D', line)):
            line += str(self.port.read(self.port.in_waiting))[2:-1]
        result = int(re.search(r'S=-?\d+', line).group()[2:])
        return result
    
    def readRPMs(self):
        left = self.readRPM(self.leftChannel)
        right = self.readRPM(self.rightChannel)
        return left,right
        #endregion
