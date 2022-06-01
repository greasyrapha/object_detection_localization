#! /usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()

config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

count = 0

try:
    while True:
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

#        depth_pixel1 = [130, 90]
#        depth_pixel2 = [130, 240]
#        depth_pixel3 = [130, 390]
#        depth_pixel4 = [320, 90]
        depth_pixel5 = [320, 240]
#        depth_pixel6 = [320, 390]
#        depth_pixel7 = [510, 90]
#        depth_pixel8 = [510, 240]
#        depth_pixel9 = [510, 390]
#        for i in range(0, 640):
#            for j in range(0, 480):
#                if depth_frame.get_distance(i, j) != 0:
#                    count = count + 1
                    

#        depth_point1 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel1, depth_frame.get_distance(130,90))
#        depth_point2 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel2, depth_frame.get_distance(130,240))
#        depth_point3 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel3, depth_frame.get_distance(130,390))
#        depth_point4 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel4, depth_frame.get_distance(320,90))
        depth_point5 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel5, depth_frame.get_distance(320,240))
#        depth_point6 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel6, depth_frame.get_distance(320,390))
#        depth_point7 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel7, depth_frame.get_distance(510,90))
#        depth_point8 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel8, depth_frame.get_distance(510,240))
#        depth_point9 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel9, depth_frame.get_distance(510,390))

#        depth_point_ros1 = [round(depth_point1[2], 4), -round(depth_point1[0], 4), -round(depth_point1[1], 4)]
#        depth_point_ros2 = [round(depth_point2[2], 4), -round(depth_point2[0], 4), -round(depth_point2[1], 4)]
#        depth_point_ros3 = [round(depth_point3[2], 4), -round(depth_point3[0], 4), -round(depth_point3[1], 4)]
#        depth_point_ros4 = [round(depth_point4[2], 4), -round(depth_point4[0], 4), -round(depth_point4[1], 4)]
        depth_point_ros5 = [round(depth_point5[2], 4), -round(depth_point5[0], 4), -round(depth_point5[1], 4)]
#        depth_point_ros6 = [round(depth_point6[2], 4), -round(depth_point6[0], 4), -round(depth_point6[1], 4)]
#        depth_point_ros7 = [round(depth_point7[2], 4), -round(depth_point7[0], 4), -round(depth_point7[1], 4)]
#        depth_point_ros8 = [round(depth_point8[2], 4), -round(depth_point8[0], 4), -round(depth_point8[1], 4)]
#        depth_point_ros9 = [round(depth_point9[2], 4), -round(depth_point9[0], 4), -round(depth_point9[1], 4)]


#        print(depth_point_ros5)
#        print(count)
#        count = 0
        print(depth_frame.get_distance(320, 240))
        color_image = np.asanyarray(color_frame.get_data())

#        cv2.circle(color_image, (130,90), 10, (0,0,255), 3)
#        cv2.circle(color_image, (130,240), 10, (0,0,255), 3)
#        cv2.circle(color_image, (130,390), 10, (0,0,255), 3)
#        cv2.circle(color_image, (320,90), 10, (0,0,255), 3)
        cv2.circle(color_image, (320,240), 10, (0,0,255), 3)
#        cv2.circle(color_image, (320,390), 10, (0,0,255), 3)
#        cv2.circle(color_image, (510,90), 10, (0,0,255), 3)
#        cv2.circle(color_image, (510,240), 10, (0,0,255), 3)
#        cv2.circle(color_image, (510,390), 10, (0,0,255), 3)

#        cv2.putText(color_image, str(depth_point_ros1), (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
#        cv2.putText(color_image, str(depth_point_ros2), (30, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
#        cv2.putText(color_image, str(depth_point_ros3), (30, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
#        cv2.putText(color_image, str(depth_point_ros4), (220, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        cv2.putText(color_image, str(depth_point_ros5), (220, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
#        cv2.putText(color_image, str(depth_point_ros6), (220, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
#        cv2.putText(color_image, str(depth_point_ros7), (410, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
#        cv2.putText(color_image, str(depth_point_ros8), (410, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
#        cv2.putText(color_image, str(depth_point_ros9), (410, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)


        cv2.imshow('color', color_image)
        cv2.waitKey(1)
finally:
    pipeline.stop()
