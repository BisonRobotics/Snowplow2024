import numpy
import math

from typing import List
from obstacle_detection import Point

class Segment:
    def __init__(self, p1: Point, p2: Point):
        if p1.cross(p2) > 0.0:
            self._p1, self._p2 = p1, p2
        else:
            self._p1, self._p2 = p2, p1

    def setFirstPoint(self, x: float, y: float): 
        self._p1.x = x; 
        self._p1.y = y

    def setLastPoint(self, x: float, y: float):
        self._p2.x = x
        self._p2.y = y

    def length(self) -> float:
        return (self._p2 - self._p1).length()
    
    def normal(self) -> Point:
        return (self._p2 - self._p1).perpendicular().normalize()

    def first_point(self):
        return self._p1
    
    def last_point(self):
        return self._p2
    
    def projection(self, p: Point) -> Point:
        a: Point = self._p2 - self._p1
        b: Point = p - self._p1

        return self._p1 + a.dot(b) * a / a.lengthSquare()

    # std::list<Point>& point_set() { return point_set_; }
    # def point_set(self) -> List[Point]:
    #     return self.point_set

    def distanceTo(self, p: Point) -> float:
        return (p - self.projection(p)).length()

    def __str__(self) -> str:
        return f"p1: {self._p1}, p2: {self._p2}"