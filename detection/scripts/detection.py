#! /usr/bin/env python3

import roslib
import rospy
import tf
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from visualization_msgs.msg import Marker

import argparse
import math
import os
import os.path as osp
import sys
import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn
import pyrealsense2 as rs

from utils.general import non_max_suppression, save_one_box, scale_coords, set_logging, xyxy2xywh
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, time_sync
from utils.augmentations import letterbox
from models.experimental import attempt_load
#from cv_bridge import CvBridge, CvBridgeError

ROOT = os.getcwd()
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from pathlib import Path
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

class Detector:
    def __init__(self):
#        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()

        self.stream_width = 640
        self.stream_height = 480
        self.stream_fps = 30

        self.camerapose = [0, 0, 0] #realsense pose for transform

        self.det_points = np.array([["name", "x", "y", "n"]])

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.stream_width, self.stream_height, rs.format.z16, self.stream_fps)
        self.config.enable_stream(rs.stream.color, self.stream_width, self.stream_height, rs.format.bgr8, self.stream_fps)
        self.profile = self.pipeline.start(self.config)

        self.subpose = rospy.Subscriber('/slam_out_pose', PoseStamped, self.poseCallback, queue_size=1)

#        self.pubresult = rospy.Publisher('/detection_result_image', Image, queue_size=1)
        self.puboptmarker = rospy.Publisher("/opt_marker", Marker, queue_size=1) 

        self.file = open(osp.join(ROOT, 'catkin_ws/src/detection/scripts/output/output.txt'), 'w')

        while True:
            with torch.no_grad():
                self.detection()

    def poseCallback(self, slamout):
        self.position = [slamout.pose.position.x, slamout.pose.position.y, slamout.pose.position.z]
        self.orientation = [slamout.pose.orientation.x, slamout.pose.orientation.y, slamout.pose.orientation.z, slamout.pose.orientation.w]
        self.br.sendTransform((self.position[0] + self.camerapose[0], self.position[1] + self.camerapose[1], self.position[2] + self.camerapose[2]), 
                              (self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]), 
                               rospy.Time.now(), "realsense", "map")

    def init_opt_marker(self):
        self.opt_marker = Marker()
        self.opt_marker.header.frame_id = "/map"
        self.opt_marker.ns = "opt_marker"
        self.opt_marker.id = 1
        self.opt_marker.type = Marker.POINTS
        self.opt_marker.action = Marker.ADD
        self.opt_marker.color = ColorRGBA(0, 0, 1, 1)
        self.opt_marker.scale = Vector3(0.2, 0.2, 0)

    def draw_opt_marker(self, x, y):
        self.opt_marker.points.append(Point(x, y, 0))

    def setpose(self, px, py):
        pose = PoseStamped()
        
        pose.header.frame_id = "realsense"

        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = 0
        
        pose.pose.orientation.x = self.orientation[0]
        pose.pose.orientation.y = self.orientation[1]
        pose.pose.orientation.z = self.orientation[2] 
        pose.pose.orientation.w = self.orientation[3]

        self.g_coord = self.ls.transformPose("map", pose)
       
    def detection(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

        color_image = np.asanyarray(color_frame.get_data())
        img0 = color_image

        color_image = letterbox(img0, new_shape=imgsz)[0]
        color_image = color_image[:, :, ::-1].transpose(2, 0, 1)
        color_image = np.ascontiguousarray(color_image)

        img = color_image
        im0s = img0
        
        dataset = [None, img, img0, None]
        
        w = str(weights)
        stride, names = 64, [f'class{i}' for i in range(1000)]
        names = model.module.names if hasattr(model, 'module') else model.names

        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))

        img = torch.from_numpy(img).to(device)
        img = img.float() 
        img = img / 255.0

        if len(img.shape) == 3:
            img = img[None]

        pred = model(img, augment=False, visualize=False)[0]
        pred = non_max_suppression(pred, 0.3, 0.45, None, 1000)

        dist = 0
        sumx = 0
        sumy = 0
        
        self.init_opt_marker()

        if self.det_points.shape[0] > 1:
            for i in range(1, self.det_points.shape[0]):
                self.draw_opt_marker(float(self.det_points[i, 1:][0]), float(self.det_points[i, 1:][1]))

        for i, det in enumerate(pred):
            p, s, im0, frame = dataset[0], '', im0s.copy(), getattr(dataset, 'frame', 0)
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  
            imc = im0
            annotator = Annotator(im0, line_width=3, example=str(names))
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                for *xyxy, conf, cls in reversed(det): 
                    c = int(cls)
  
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()

                    posx = int(xywh[0] * self.stream_width)
                    posy = int(xywh[1] * self.stream_height)

                    d_point = rs.rs2_deproject_pixel_to_point(depth_intrin, (posx,posy), depth_frame.get_distance(posx,posy))	#Depth point
                    d_point_ros = [round(d_point[2], 4), -round(d_point[0], 4), -round(d_point[1], 4)]				#Depth point for ROS
                    d_dist = d_point_ros[0] + d_point_ros[1] + d_point_ros[2]

                    label = f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    cv2.putText(im0, str(d_point_ros), (posx, posy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                    if d_dist != 0 and d_point_ros[0] < 10:
                        self.file.write(f'{names[c]} : {d_point_ros[0]}, {d_point_ros[1]}, {d_point_ros[2]}' + '\n')
                        self.setpose(d_point_ros[0], d_point_ros[1])

                        posgx = self.g_coord.pose.position.x
                        posgy = self.g_coord.pose.position.y

                        w_class = np.where(self.det_points == names[c])[0] #Find class
                        n_class = w_class.shape[0] #Number of Known class

                        knn_score = 0
                        knn_rst = 1

                        if n_class > 0:
                            for i in w_class:
                                w_x = abs(posgx - float(self.det_points[i, 1:][0])) # x
                                w_y = abs(posgy - float(self.det_points[i, 1:][1])) # y
                                dist = math.sqrt(pow(w_x, 2) + pow(w_y, 2))
                                if dist >= 0.4: #dist_threshold
                                    knn_score += 1
                                elif dist < 0.4 and dist >= 0.1:
                                    self.det_points[i, 1:][2] = int(self.det_points[i, 1:][2]) + 1
                                    self.det_points[i, 1:][0] = (float(self.det_points[i, 1:][0]) + posgx) / 2.0
                                    #self.det_points[i, 1:][0] = (float(self.det_points[i, 1:][0]) + posgx) / float(self.det_points[i, 1:][2])
                                    self.det_points[i, 1:][1] = (float(self.det_points[i, 1:][1]) + posgy) / 2.0
                                    #self.det_points[i, 1:][1] = (float(self.det_points[i, 1:][1]) + posgy) / float(self.det_points[i, 1:][2])
                                    knn_rst = 0
                                elif dist < 0.1:
                                    knn_rst = 0
                            if knn_score * knn_rst != 0:
                                self.det_points = np.append(self.det_points, [[names[c], posgx, posgy, 1]], axis=0) #add det_points
                        elif n_class == 0:
                            self.det_points = np.append(self.det_points, [[names[c], posgx, posgy, 1]], axis=0) #add det_points
                   
        cv2.imshow('Detection', im0)
        cv2.waitKey(1)

        self.puboptmarker.publish(self.opt_marker)
#        self.pubresult.publish(self.bridge.cv2_to_imgmsg(im0, encoding="bgr8"))
        self.file.write('====================================' + '\n')


def main():
    detector = Detector()

if __name__ == "__main__": 
    set_logging()
    device = ''
    device = select_device(device)
    weights = osp.join(ROOT, 'catkin_ws/src/detection/scripts/yolov5s.pt')
    imgsz = 640
    model = attempt_load(weights, map_location=device)

    rospy.init_node('detection')
    main()
