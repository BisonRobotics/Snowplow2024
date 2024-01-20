import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PointStamped

import tf2_ros as tf2

from obstacle_detection.msg import Obstacles
from obstacle_detection.point import Point
from obstacle_detection.segment import Segment
from obstacle_detection.circle import Circle

class ObstacleDetector():

    def __init__(self):
        super().__init__("obstacleDetector")
        self.updateParams()

        #Start subscriptions
        if (self.p_use_scan_):
            self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scanCallback, 10)
            self.scan_sub  # prevent unused variable warning
        else:
            self.pcl_sub = self.create_subscription(PointCloud, 'pcl', self.pclCallback, 10)
            self.pcl_sub  # prevent unused variable warning
        #Start publisher
        self.obstacles_pub = self.create_publisher(Obstacles, 'obstacles', 5)

        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)

        #Define variables
        self.initial_points = []
        self.segments = []
        self.circles = []



    def updateParams(self):
        print("TODO")

    def scanCallback(self, msg:LaserScan):
        self.initial_points.clear()
        phi = msg.angle_min - msg.angle_increment

        for r in msg.ranges:
            phi += msg.angle_increment

            if r >= msg.range_min and r <= msg.range_max and r <= self.p_max_scanner_range:
                self.initial_points.push_back(Point.fromPoolarCoords(r, phi))
        
        self.processPoints();

    def pclCallback(self,pcl:PointCloud):
        self.initial_points.clear()

        for p in pcl.points:
            if Point(p.x, p.y).lengthSquared() <= math.pow(self.p_max_scanner_range, 2.0):
                self.initial_points.push_back(Point(p.x, p.y))

        self.processPoints()

    def processPoints(self):
        self.segments_.clear()
        self.circles_.clear()

        self.groupPointsAndDetectSegments()
        self.mergeSegments()

        self.detectCircles()
        self.mergeCircles()

        if (self.p_transform_to_world):
            self.transformToWorld()

        self.publishObstacles()

    def groupPointsAndDetectSegments(self) :
        point_set = []

        for point in self.initial_points:
            if (point_set.size() != 0) :
                r = point.length()
                if ((point - point_set.back()).lengthSquared() > math.pow(self.p_max_group_distance + r * self.p_distance_proportion, 2.0)):
                    self.detectSegments(point_set);
                    point_set.clear();            
            point_set.push_back(point)        
        self.detectSegments(point_set) #Check the last point set too!

    def detectSegments(self, point_set:list):
        if (point_set.size() < self.p_min_group_points) :
            return None
        segment = Segment(Point(0.0, 0.0), Point(1.0, 0.0))
        if (self.p_use_split_and_merge):
            segment = self.fitSegment(point_set);
        else: 
            # Use Iterative End Point Fit
            segment = Segment(point_set.front(), point_set.back())

        set_divider = []
        max_distance = 0
        distance = 0

        #Seek the point of division; omit first and last point of the set
        for point in point_set:
            if point != point_set[0] or point != point_set[-1]:
                distance = segment.distanceTo(point)
                if distance>= max_distance:
                    r = point.length()
                    if distance > self.p_max_split_distance_ + r * self.p_distance_proportion:
                        max_distance = distance
                        set_divider = point_set[point:]

        if (max_distance > 0.0): # Split the set
            point_set.insert(set_divider, *set_divider)#Clone the dividing point for each group

            subset1  = list()
            subset2 = list()
            subset1.splice(subset1[0], point_set, point_set[0], set_divider)
            subset2.splice(subset2[0], point_set, set_divider, point_set[-1])

            self.detectSegments(subset1)
            self.detectSegments(subset2)
        else  : # Add the segment
            if not self.p_use_split_and_merge:
                segment = self.fitSegment(point_set)

            if (segment.length() > 0.0):
                self.segments.push_back(segment)
                self.segments[-1].point_set().assign(point_set[0], point_set[-1])
    
    def mergeSegments(self):
        i = 0
        while i < len(self.segments):
            j = i + 1
            while j< len(self.segments):
                if self.compareAndMergeSegments(self.segments(i),self.segments(j)):
                    temp_ptr = i
                    self.segments.insert(i,self.segments[-1])
                    self.segments.pop()
                    self.segments.pop(temp_ptr)
                    self.segments.pop(j)
                    if i > 0 :
                        i -= 1
                    break
                j += 1
            i +=1
    
    def compareAndMergeSegments(self,s1:Segment,s2:Segment)->bool:
        if s1 == s2:
            return False
        
        #Segments must be provided counter-clockwise
        if (s1.first_point().cross(s2.first_point()) < 0.0):
            return self.compareAndMergeSegments(s2, s1)
        
        if ((s1.last_point() - s2.first_point()).length() < self.p_max_merge_separation) :
            merged_points = list()
            merged_points.insert(merged_points[0], s1.point_set().begin(), s1.point_set().end());
            merged_points.insert(merged_points.end(), s2.point_set().begin(), s2.point_set().end());

            s = self.fitSegment(merged_points)

            if (s.distanceTo(s1.first_point()) < self.p_max_merge_spread and 
                s.distanceTo(s1.last_point())  < self.p_max_merge_spread and
                s.distanceTo(s2.first_point()) < self.p_max_merge_spread and
                s.distanceTo(s2.last_point())  < self.p_max_merge_spread) :
                self.segments.push_back(s)
                self.segments[-1].point_set().assign(merged_points[0], merged_points[-1])
                return True                

        return False

    def detectCircles(self):
        for s in self.segments:
            c = Circle(s)
            c.setRadius(c.radius() + self.p_radius_enlargement)

            if (c.radius() < self.p_max_circle_radius):
                self.circles.push_back(c)
                
    def mergeCircles(self):
        i = 0
        while i < len(self.circles):
            j = i + 1
            while j< len(self.circles):
                if self.compareAndMergeCircles(self.circles(i),self.circles(j)):
                    temp_ptr = i
                    self.circles.insert(i,self.circles[-1])
                    self.circles.pop()
                    self.circles.pop(temp_ptr)
                    self.circles.pop(j)
                    if i > 0 :
                        i -= 1
                    break
                j += 1
            i +=1

    def compareAndMergeCircles(self, c1:Circle,c2:Circle):
        #If circle c1 is fully inside c2 - merge and leave as c2
        if (c1.radius() + (c2.center() - c1.center()).length() <= c2.radius()):
            self.circles.push_back(c2)
            return True
        
        #If circle c2 is fully inside c1 - merge and leave as c1
        if (c2.radius() + (c2.center() - c1.center()).length() <= c1.radius()) :
            self.circles.push_back(c1)
            return True
        
        # If circles intersect and are 'small' - merge
        if (c1.radius() + c2.radius() >= (c2.center() - c1.center()).length()) :
            s=Segment(c1.center(), c2.center())
            c = Circle(s)
            c.setRadius(c.radius() +max(c1.radius(), c2.radius()))

            if (c.radius() < self.p_max_circle_radius) :
                self.circles.push_back(c)
                return True
                    
        return False

    def transformToWorld(self):
        point_l = PointStamped()
        point_w = PointStamped()

        point_l.header.stamp = rclpy.Time.now()
        point_l.header.frame_id = self.p_scanner_frame

        point_w.header.stamp = rclpy.Time.now()
        point_w.header.frame_id = self.p_world_frame

        try:
            self.tf_listener.callback

        except  Exception:
            self.get_logger().error("%s",ex.what())







def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetector()
    rclpy.spin(node)
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
