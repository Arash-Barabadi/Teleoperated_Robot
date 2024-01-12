import sys
import os 
import time
import struct

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from mtlt305_ros.MTLT305 import MTLT305

class mtlt305_node(Node):
    
    def __init__(self):
        super().__init__("mtlt305_node")
        
        self.mtlt305 = MTLT305(port="/dev/ttyTHS0", baudrate=57600)  # on jetson /dev/ttyTHS0 on PC with USB-Serial connection /dev/ttyUSB0
        if self.mtlt305.port is None:
            self.get_logger().fatal("MTLT305 serial-port not valid")
            exit()
            return
        
        self.mtlt305.get_packet_request(packet="ID")
        while not self.mtlt305.received_id:
            time.sleep(0.05)
            self.mtlt305.read_packet_response()
            pass

        self.mtlt305.get_packet_request(packet="VR")
        while not self.mtlt305.received_vr:
            time.sleep(0.05)
            self.mtlt305.read_packet_response()
            pass

        self.get_logger().info(f"Connected to MTLT305: \nPort: \t {self.mtlt305.port.name}\nID: \t\t {self.mtlt305.id}\nVersion: \t {self.mtlt305.VR}")
        
        self._publisher = self.create_publisher(Imu, "/MTLT305/imu", 10)
        self._timer = self.create_timer(1e-3, self.timer_callback)
        self._watchdog = self.create_timer(4, self.watchdog_callback)
        
    def timer_callback(self):
        self.mtlt305.read_packet_response()
        if self.mtlt305.received_ang2:
            self._watchdog.reset()
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'mtlt305'
            rot = Rotation.from_euler("xyz", [self.mtlt305.a2.rollAngle, self.mtlt305.a2.pitchAngle, self.mtlt305.a2.yawAngleTrue], degrees=True)
            [x, y, z, w] = rot.as_quat()
            msg.orientation.x = x
            msg.orientation.y = y
            msg.orientation.z = z
            msg.orientation.w = w
            msg.orientation_covariance = list(np.zeros((9,),dtype=np.float64))

            msg.angular_velocity.x = self.mtlt305.a2.rollRate
            msg.angular_velocity.y = self.mtlt305.a2.pitchRate
            msg.angular_velocity.z = self.mtlt305.a2.yawRate
            msg.angular_velocity_covariance = list(np.zeros((9,),dtype=np.float64))

            msg.linear_acceleration.x = self.mtlt305.a2.x_accel
            msg.linear_acceleration.y = self.mtlt305.a2.y_accel
            msg.linear_acceleration.z = self.mtlt305.a2.z_accel
            msg.linear_acceleration_covariance = list(np.zeros((9,),dtype=np.float64))
            self._publisher.publish(msg)
            self.get_logger().info("transmitted IMU-msg")
        
    def watchdog_callback(self):
        self.get_logger().fatal("Connection Lost")
        exit()
        
def main(args=None):
    rclpy.init(args=args)
    node = mtlt305_node()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
