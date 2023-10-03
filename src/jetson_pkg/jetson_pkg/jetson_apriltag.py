import rclpy
from rclpy.node import Node

from std_msgs.ms import Float64MultiArray

import cv2
import apriltag

cap = cv2.VideoCapture('rtsp://admin:password@192.168.1.131:80/cam/realmonitor?channel=1&subtype=0')
detector = apriltag.Detector()

class ApriltagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/apriltag', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = Float64MultiArray()
        success, frame = cap.read()
        if success:
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            detections = detector.detect(gray)
            result = []
            result.append(float(detections[0].tag_id))
            for x in detections[0].center:
                result.append(x)
            for x in detections[0].corners:
                for y in x:
                    result.append(y)
            msg.data = result
        else:
            msg.data = []
        self.publisher_.publish(msg)
        self.get_logger().info('publishing: %s' % str(msg.data))
        
def main(args=None):
    rclpy.init(args=args)
    apriltag_publisher = ApriltagPublisher()
    rclpy.spin(apriltag_publisher)
    
    apriltag_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()