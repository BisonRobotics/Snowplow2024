import rclpy
from rclpy.node import Node
from vnpy import *
from std_msgs.msg import String
from sensor_msgs.msg import Imu

class IMU (Node):
    s = VnSensor()

    def __init__(self):
        super().__init__('imu_node')
        self.s.connect('/dev/ttyUSB0', 115200)
        self.publisher_=self.create_publisher(Imu, 'imu',10)
        timer_period=0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Imu()
        wxyz = self.s.read_yaw_pitch_roll()
        msg.orientation.x = wxyz.y
        self.publisher_.publish(msg)
        self.get_logger().info('publsihing')

def main(args=None):
    rclpy.init(args=args)

    imu = IMU()
    rclpy.spin(imu)

    imu.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
        