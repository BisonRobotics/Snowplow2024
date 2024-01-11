import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

import math
import numpy as np
import threading
import cv2
import apriltag

cap = cv2.VideoCapture('rtsp://admin:hyflex@192.168.1.131:80/cam/realmonitor?channel=1&subtype=0')
detector = apriltag.Detector()
fx, fy, cx, cy = (1071.1362274102335, 1102.1406887400624, 953.030188084331, 468.0382502048589)

class ApriltagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')
        self.publisher_ = self.create_publisher(Point, '/apriltag', 10)
        timer_period = 0.5
        self.latest_frame = None
        threading.Thread(target=self.keep_up_thread).start()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def keep_up_thread(self):
        while True:
            success, frame = cap.read()
            if success:
                self.latest_frame = frame

    def timer_callback(self):
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_RGB2GRAY)
        detections = detector.detect(gray)
        if len(detections) > 0:
            pose, _, _ = detector.detection_pose(detections[0], [fx,fy,cx,cy], 0.3254375)
            self.get_logger().info(f'x={pose[0][3]} meters; y={pose[1][3]} meters; z={pose[2][3]} meters; rotation={np.arcsin(-pose[2][0]) * (180 / math.pi)} degrees')
            msg = Point()
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
            self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    apriltag_publisher = ApriltagPublisher()
    rclpy.spin(apriltag_publisher)
    
    apriltag_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
