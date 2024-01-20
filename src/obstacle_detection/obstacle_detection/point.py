import numpy
import math

from obstacle_detection.point import Point

class Point:

    def __init__(self, x :float = 0.0, y :float = 0.0):
        self.x = x
        self.y = y

    def clone(self)-> Point:
        return Point(self.x, self.y)
    
    def fromPoolarCoords(self,r: float, phi: float) -> Point:
        return Point(r * math.cos(phi), r * math.sin(phi))
    
    def length(self) -> float:
        return math.sqrt(math.pow(self.x, 2.0) + math.pow(self.y, 2.0))
    
    def lengthSquared(self) -> float:
        return math.pow(self.x, 2.0) + math.pow(self.y, 2.0)
    
    def angle(self) -> float:
        return math.atan2(self.y, self.x)
    
    def angleDeg(self) -> float:
        return 180.0 * math.atan2(self.y, self.x) / math.pi
    
    def  dot(self, p: Point) -> float:
        return self.x * p.x + self.y * p.y
    
    def cross(self, p: Point) -> float:
         return self.x * p.y - self.y * p.x
    
    def normalize(self) -> Point:
        L = self.length()
        if (L > 0.0):
            self.x /= L
            self.y /= L
        return self
    
    def reflected(self, normal:Point)-> Point:
        return self - 2.0 * normal * (normal.dot(self))
    
    def perpendicular(self) -> Point:
        return Point(-self.y, self.x)
    
    def __add__ (self, p1:Point, p2:Point):
        return Point(p1.x + p2.x, p1.y + p2.y)
    
    def __sub__(self,p1:Point, p2:Point):
        return Point(p1.x - p2.x, p1.y - p2.y)
    
    def __mul__(self, f:float, p:Point): 
        return Point(f * p.x, f * p.y)
    
    def __mul__(self,p:Point, f:float):
        return Point(f * p.x, f * p.y)
    
    def __truediv__ (self, p:Point, f:float):
        if (f!=0.0):
            return Point(p.x / f, p.y / f)
        return Point()
    
    def __sub__(self):
        return Point(-self.x, -self.y)
    
    def __add__(self):
        return Point( self.x, self.y)
    
    def __iadd__(self,p:Point) :
        self.x += p.x; self.y += p.y
        return self
    
    def __isub__(self, p:Point) :
        self.x -= p.x; self.y -= p.y
        return self
    
    def __eq__(self,p1:Point, p2:Point) :
        return (p1.x == p2.x and p1.y == p2.y)
    
    def __ne__ (self, p1:Point, p2:Point) :
        return not(p1 == p2)
    
    def __lt__(self, p1:Point, p2:Point):
        return (p1.lengthSquared() < p2.lengthSquared())
    
    def __le__(self, p1:Point, p2:Point):
        return (p1.lengthSquared() <= p2.lengthSquared())
    
    def __gt__(self, p1:Point, p2:Point) :
        return (p1.lengthSquared() > p2.lengthSquared())
    
    def __ge__(self, p1:Point, p2:Point):
        return (p1.lengthSquared() >= p2.lengthSquared())
    
    def __invert__(self, p1:Point, p2:Point):
        return (p1.lengthSquared() >= p2.lengthSquared())
    
    


