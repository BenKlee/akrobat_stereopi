HOST = 'localhost'
PORT = 4223
UID = '6fZG8m' # Change XXYYZZ to the UID of your IMU Brick 2.0

from math import radians
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

class IMUNode(Node):

    def __init__(self, publish_frequency_Hz: float):
        self.__publish_frequency_Hz = publish_frequency_Hz
        self.__imu_publisher = self.create_publisher(Imu, '/imu/data', 10)

    def __enter__(self):
        self.__ip_connection = IPConnection() # Create IP connection
        self.__imu = BrickIMUV2(UID, self.__ip_connection) # Create device object

        self.__ip_connection.connect(HOST, PORT) # Connect to brickd
        # Don't use device before ipcon is connected

        self.__imu.register_callback(self.__imu.CALLBACK_ALL_DATA, self.publish_data)
        self.__imu.set_all_data_period((1/self.__publish_frequency_Hz)*1000)

    def __exit__(self):
        self.__ip_connection.disconnect()
        self.destroy_node()

    def publish_data(self, acceleration, magnetic_field, angular_velocity, euler_angle, quaternion,
                linear_acceleration, gravity_vector, temperature, calibration_status):

        self.get_logger().info('Acceleration [X]: ' + str(acceleration[0]/100.0) + ' m/s²')
        self.get_logger().info('Acceleration [Y]: ' + str(acceleration[1]/100.0) + ' m/s²')
        self.get_logger().info('Acceleration [Z]: ' + str(acceleration[2]/100.0) + ' m/s²')
        self.get_logger().info('Magnetic Field [X]: ' + str(magnetic_field[0]/16.0) + ' µT')
        self.get_logger().info('Magnetic Field [Y]: ' + str(magnetic_field[1]/16.0) + ' µT')
        self.get_logger().info('Magnetic Field [Z]: ' + str(magnetic_field[2]/16.0) + ' µT')
        self.get_logger().info('Angular Velocity [X]: ' + str(angular_velocity[0]/16.0) + ' °/s')
        self.get_logger().info('Angular Velocity [Y]: ' + str(angular_velocity[1]/16.0) + ' °/s')
        self.get_logger().info('Angular Velocity [Z]: ' + str(angular_velocity[2]/16.0) + ' °/s')
        self.get_logger().info('Euler Angle [Heading]: ' + str(euler_angle[0]/16.0) + ' °')
        self.get_logger().info('Euler Angle [Roll]: ' + str(euler_angle[1]/16.0) + ' °')
        self.get_logger().info('Euler Angle [Pitch]: ' + str(euler_angle[2]/16.0) + ' °')
        self.get_logger().info('Quaternion [W]: ' + str(quaternion[0]/16383.0))
        self.get_logger().info('Quaternion [X]: ' + str(quaternion[1]/16383.0))
        self.get_logger().info('Quaternion [Y]: ' + str(quaternion[2]/16383.0))
        self.get_logger().info('Quaternion [Z]: ' + str(quaternion[3]/16383.0))
        self.get_logger().info('Linear Acceleration [X]: ' + str(linear_acceleration[0]/100.0) + ' m/s²')
        self.get_logger().info('Linear Acceleration [Y]: ' + str(linear_acceleration[1]/100.0) + ' m/s²')
        self.get_logger().info('Linear Acceleration [Z]: ' + str(linear_acceleration[2]/100.0) + ' m/s²')
        self.get_logger().info('Gravity Vector [X]: ' + str(gravity_vector[0]/100.0) + ' m/s²')
        self.get_logger().info('Gravity Vector [Y]: ' + str(gravity_vector[1]/100.0) + ' m/s²')
        self.get_logger().info('Gravity Vector [Z]: ' + str(gravity_vector[2]/100.0) + ' m/s²')
        self.get_logger().info('Temperature: ' + str(temperature) + ' °C')
        self.get_logger().info('Calibration Status: ' + format(calibration_status, '08b'))
        self.get_logger().info('')

        imu_message = Imu()

        imu_message.linear_acceleration = Vector3(x=linear_acceleration[0]/100.0, y=linear_acceleration[1]/100.0, z=linear_acceleration[2]/100.0)

        imu_message.angular_velocity = Vector3(x=radians(angular_velocity[0]/16.0), y=radians(angular_velocity[1]/16.0), z=radians(angular_velocity[2]/16.0))

        imu_message.orientation = Quaternion(w=quaternion[0]/16383.0, x=quaternion[1]/16383.0, y=quaternion[2]/16383.0, z=quaternion[3]/16383.0)

        self.__imu_publisher.publish(imu_message)

def main(args=None):
    rclpy.init(args=args)

    with IMUNode(publish_frequency_Hz=60) as imu:
        rclpy.spin(imu)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
