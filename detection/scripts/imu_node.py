#! /usr/bin/env python3

import roslib
import rospy
import numpy
from sensor_msgs.msg import Imu

import numpy as np
import pyrealsense2 as rs

class Camera_Imu:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200) #Enable accelerometer 62.5, 250(Hz)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200) #Enable gyroscope 200, 400(Hz)
        self.profile = self.pipeline.start(self.config)

        self.pubimu = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)
        
        print("IMU Node is UP!")

        try:
            while True:
                self.imu_1()
        finally:
            self.pipeline.stop()

    def setImu(self, x, y, z, roll, pitch, yaw):
        self.imu = Imu()
        self.imu.header.frame_id = '/imu_link'
        self.imu.angular_velocity.x = roll
        self.imu.angular_velocity.y = pitch
        self.imu.angular_velocity.z = yaw
        self.imu.linear_acceleration.x = x
        self.imu.linear_acceleration.y = y
        self.imu.linear_acceleration.z = z

    def imu_1(self):
        frames = self.pipeline.wait_for_frames()

        accel_frame = frames[0].as_motion_frame().get_motion_data()
        gyro_frame = frames[1].as_motion_frame().get_motion_data()

        accel = np.asarray([accel_frame.x, accel_frame.y, accel_frame.z])
        gyro = np.asarray([gyro_frame.x, gyro_frame.y, gyro_frame.z])

        #print("accel : ", accel)
        #print("gyro : ", gyro)

        self.setImu(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2])
        self.pubimu.publish(self.imu)

def main():
    camera_imu = Camera_Imu()

if __name__ == "__main__": 
    rospy.init_node('imu')
    main()
