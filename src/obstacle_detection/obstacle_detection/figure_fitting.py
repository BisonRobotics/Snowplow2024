import numpy as np
import pyarma as pa
import math
from obstacle_detection.segment import Segment
from obstacle_detection.point import Point
from obstacle_detection.circle import Circle

class FigureFitting():
    @staticmethod 
    def fitSegment(self, *point_set) -> Segment:
        """Returns a total best fit approximation of segment based on given point set. The equation used for fitting is given by:
                    Ax + By = -C
            and the A, B, C parameters are normalized."""
        n = point_set.count
        input = pa.mat(n,2,pa.fill.zeros)
        output = pa.vec(n,pa.fill.ones)
        params = pa.vec(2,pa.fill.zero)

        i = 0
        for point in point_set:
            input[i, 0] = point.x
            input[i, 1] = point.y
            i += 1

        #Find A and B coefficients from linear regression (assuming C = -1.0)
        params = pa.pinv(input) * output

        A = params(0)
        B = params(1)
        C = -1.0
        D = (A * A + B * B)

        # Find end points
        p1 = point_set.front()
        p2 = point_set.back()

        if (D > 0.0) :  # Project end points on the line
            projected_p1 = Point()
            projected_p1.x = ( B * B * p1.x - A * B * p1.y - A * C) / D
            projected_p1.y = (-A * B * p1.x + A * A * p1.y - B * C) / D

            projected_p2 = Point()
            projected_p2.x = ( B * B * p2.x - A * B * p2.y - A * C) / D
            projected_p2.y = (-A * B * p2.x + A * A * p2.y - B * C) / D

            return Segment(projected_p1, projected_p2)
        else:
            return Segment(p1, p2)
        
    @staticmethod 
    def fitCircle(self,*point_set) -> Circle:
        """Returns a total best fit approximation of a circle based on given point set. The equation used for fitting is given by:
                a1 * x + a2 * y + a3 = -(x^2 + y^2)
           where parameters a1, a2, a3 are obtained from circle equation:
                (x-x0)^2 + (y-y0)^2 = r^2."""
        n = point_set.count

        input = pa.mat(n,3,pa.fill.zeros)
        output = pa.vec(n,pa.fill.ones)
        params = pa.vec(3,pa.fill.zero)

        i = 0
        for point in point_set:
            input[i, 0] = point.x
            input[i, 1] = point.y
            input[i, 2] = 1.0

            output[i] = -(math.pow(point.x, 2) + math.pow(point.y, 2))
            i += 1
        
        #Find a_1, a_2 and a_3 coefficients from linear regression
        params = pa.pinv(input) * output

        center = Point(-params(0) / 2.0, -params(1) / 2.0)
        radius =  math.sqrt((params(0) * params(0) + params(1) * params(1)) / 4.0 - params(2))
        
        return Circle(center, radius)
