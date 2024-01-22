import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from jetson_pkg.apriltag_interpretation import apriltag_interpretation

import math
import numpy as np
import threading
import cv2
import apriltag
import json

cap = cv2.VideoCapture('rtsp://admin:hyflex@192.168.1.131:80/cam/realmonitor?channel=1&subtype=0')
detector = apriltag.Detector()
fx, fy, cx, cy = (1071.1362274102335, 1102.1406887400624, 953.030188084331, 468.0382502048589)

pivot_x_offset = -0.017
pivot_z_offset = 0.83

class ApriltagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')
        self.publisher_ = self.create_publisher(Point, '/apriltag', 10)
        timer_period = 0.5
        self.latest_frame = None
        threading.Thread(target=self.keep_up_thread).start()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        with open('apriltag_poses.json') as json_file:
            self.apriltag_poses = json.load(json_file)

    def keep_up_thread(self):
        while True:
            success, frame = cap.read()
            if success:
                self.latest_frame = frame

    def timer_callback(self):
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_RGB2GRAY)
        detections = detector.detect(gray)
        if len(detections) > 0:
            msg = Twist()
            msg.linear.x = 0
            msg.linear.z = 0
            msg.angular.y = 0
            
            for detection in detections:
                pose, _, _ = detector.detection_pose(detection, [fx, fy, cx, cy], 0.3254375)
                
                relative_x = pose[0][3] + pivot_x_offset
                relative_y = pose[2][3] + pivot_z_offset
                relative_rotation = np.arcsin(-pose[2][0]) * (180 / math.pi)
                
                apriltag_pose = self.apriltag_poses[detection.family][detection.id]
                
                xr, zr, thetar = apriltag_interpretation(apriltag_pose['x'], apriltag_pose['z'], apriltag_pose['angle'], relative_x, relative_y, relative_rotation)
                
                msg.linear.x += xr
                msg.linear.z += zr
                msg.angular.y += thetar
                
            msg.linear.x /= len(detections)
            msg.linear.z /= len(detections)
            msg.angular.y /= len(detections)
            
            self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    apriltag_publisher = ApriltagPublisher()
    rclpy.spin(apriltag_publisher)
    
    apriltag_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
