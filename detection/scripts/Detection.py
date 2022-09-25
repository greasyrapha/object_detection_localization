# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
#! /usr/bin/env python3

import roslib
import rospy
from detection.msg import Object
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

ROOT = os.getcwd()
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from pathlib import Path
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

class Detector:
    def __init__(self):
        ##Tested in Intel Realsense D455
        self.stream_width = 640
        self.stream_height = 480
        self.stream_fps = 30

        ##Create pipeline
        self.pipeline = rs.pipeline()

        ##Setting configuration
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.stream_width, self.stream_height, rs.format.z16, self.stream_fps)
        self.config.enable_stream(rs.stream.color, self.stream_width, self.stream_height, rs.format.bgr8, self.stream_fps)

        ##Start pipeline
        self.profile = self.pipeline.start(self.config)

        ##Publisher for Object
        self.pubobj = rospy.Publisher("/det_out_obj", Object, queue_size=1)

        while True:
            with torch.no_grad():
                self.detection()

    def setObject(self, x, y, z, cls):
        self.obj = Object()
        self.obj.class_name = cls
        self.obj.x = x
        self.obj.y = y
        self.obj.z = z

    def detection(self):
        frames = self.pipeline.wait_for_frames()

        ##Get depth and color frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        ##Get camera parameter
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        #depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

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

                    ##Position in Image coordinate system
                    posx = int(xywh[0] * self.stream_width)
                    posy = int(xywh[1] * self.stream_height)

                    ##Local position in Image coordinate system
                    d_point = rs.rs2_deproject_pixel_to_point(depth_intrin, (posx,posy), depth_frame.get_distance(posx,posy))
                    
                    ##Image to ROS coordinate system
                    d_point_ros = [round(d_point[2], 4), -round(d_point[0], 4), -round(d_point[1], 4)]

                    label = f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    cv2.putText(im0, str(d_point_ros), (posx, posy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                    self.setObject(d_point_ros[0], d_point_ros[1], d_point_ros[2], names[c])
                    self.pubobj.publish(self.obj)
                   
        cv2.imshow('Detection', im0)
        cv2.waitKey(1)

def main():
    detector = Detector()

if __name__ == "__main__": 
    set_logging()
    device = ''
    device = select_device(device)
    weights = osp.join(ROOT, 'catkin_ws/src/detection/scripts/weights/yolov5s.pt')
    imgsz = 640
    model = attempt_load(weights, map_location=device)

    rospy.init_node('Detection')
    main()
