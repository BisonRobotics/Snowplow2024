import rclpy
from rclpy.node import Node
from vnpy import *
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature

class IMU (Node):
    s = VnSensor()

    def __init__(self):
        super().__init__('imu_node')
        self.s.connect('/dev/ttyUSB0', 115200)
        self.publisher_= self.create_publisher(Imu, 'imu',10)
        timer_period=0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        imu_msg = Imu()
        wxyz = self.s.read_attitude_quaternion()
        print(wxyz)
        imu_msg.orientation.x = wxyz.x
        imu_msg.orientation.y = wxyz.y
        imu_msg.orientation.w = wxyz.w
        imu_msg.orientation.z = wxyz.z


        angrate = self.s.read_angular_rate_measurements()
        imu_msg.angular_velocity.x = angrate.x
        imu_msg.angular_velocity.y = angrate.y
        imu_msg.angular_velocity.z = angrate.z

        accel = self.s.read_acceleration_measurements()
        imu_msg.linear_acceleration.x = accel.x
        imu_msg.linear_acceleration.y = accel.y
        imu_msg.linear_acceleration.z = accel.z

        
        self.publisher_.publish(imu_msg)
        self.get_logger().info('publsihing')

def main(args=None):
    rclpy.init(args=args)

    imu = IMU()
    rclpy.spin(imu)

    imu.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
        