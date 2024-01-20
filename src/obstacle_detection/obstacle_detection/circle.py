import numpy
from obstacle_detection.point import Point
from obstacle_detection.segment import Segment

class circle:
    

    def __init__(self, p:Point, r:float):
        self.center_ = p
        self.radius_ = r
        self.point_set_ = []

    def __init__(self,s:Segment):
        self.radius_ = 0.5773502 * s.length();  # sqrt(3)/3 * length
        self.center_ = (s.first_point() + s.last_point() - self.radius_ * s.normal()) / 2.0
        self.point_set_ = []
    
    def setRadius(self,r:float):
        self.radius_ = r

    def setCenter(self,x:float, y:float): 
        self.center_.x = x
        self.center_.y = y

    def center(self) -> Point:
        return self.center_
    
    def radius(self) -> float:
        return self.radius_
    
    def distanceTo(self, p:Point) -> float:
        return (p - self.center_).length() - self.radius_
    
    def point_set(self):
        return self.point_set_
    
    def printout(self):
        print("C" + self.center_ + "R" + self.radius_)


    


