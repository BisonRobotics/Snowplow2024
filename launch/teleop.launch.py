from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os.path

def generate_launch_description():
    default_joy_address = '/dev/input/js0'
    joy_name = LaunchConfiguration("joy_name")

    #Joystick address option
    declare_joy_address = DeclareLaunchArgument(
        name='joy_name',
        default_value=default_joy_address,
        description="Full path to Joystick Device"
    )

    #Base Joystick node
    start_joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{'dev_name':joy_name,'deadzone':0.0}]
    )

    #Joystick conversion for Hyflex node
    start_joy_conv_node = Node(
        package='joy_conv',
        executable='joy_conv',
        name='joy_conv'
    )

    #Axle manager for Hyflex node
    start_axle_manager_node = Node(
        package='axle_manager',
        executable='axle_manager',
        name='axle_manager',
        arguments=['--params-file '+os.path.join(get_package_share_directory('axle_manager'),'config','params.yaml')]
    )

    #VN100 Sensor IMU Node
    start_imu_node = Node(
        package='sensors_pkg',
        executable='imu',
        name='imu'
    )



    #Declare launch description and populate
    ld = LaunchDescription()

    #declare launch options (if any)
    ld.add_action(declare_joy_address)

    #declare launch actions
    ld.add_action(start_axle_manager_node)
    ld.add_action(start_joy_conv_node)
    ld.add_action(start_joy_node)

    return ld