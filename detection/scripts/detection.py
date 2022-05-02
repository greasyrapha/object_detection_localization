#! /usr/bin/env python3

import roslib
import rospy
from sensor_msgs.msg import Image

import argparse
import os
import sys
import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn

from utils.general import non_max_suppression, save_one_box, scale_coords, set_logging, xyxy2xywh
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, time_sync
from utils.augmentations import letterbox
from models.experimental import attempt_load
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2
from pathlib import Path
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

class Detector:
    def __init__(self, color_image_topic, depth_image_topic):
        self.bridge = CvBridge()
        self.subcolor = rospy.Subscriber(color_image_topic, Image, self.imageColorCallback, queue_size=1, buff_size=52428800)
        self.subdepth = rospy.Subscriber(depth_image_topic, Image, self.imageDepthCallback, queue_size=1, buff_size=52428800)

    def imageColorCallback(self, colordata):
        callback_colorimage = self.bridge.imgmsg_to_cv2(colordata, colordata.encoding)
        callback_colorimage = callback_colorimage[...,::-1]
        with torch.no_grad():
            self.detection(callback_colorimage)

    def imageDepthCallback(self, depthdata):
        self.callback_depthimage = self.bridge.imgmsg_to_cv2(depthdata, depthdata.encoding)

    def Loadimage(self, img):
        cap = None
        path = None
        img0 = img
        img = letterbox(img0, new_shape=640)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        return path, img, img0, cap

    def detection(self, image):
        dataset = self.Loadimage(image)

        path = dataset[0]
        img = dataset[1]
        im0s = dataset[2]
        vid_cap = dataset[3]
    
        w = str(weights)
        stride, names = 64, [f'class{i}' for i in range(1000)]  # assign defaults
        names = model.module.names if hasattr(model, 'module') else model.names
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))

        dt, seen = [0.0, 0.0, 0.0], 0

        t1 = time_sync()

        img = torch.from_numpy(img).to(device)
        img = img.float() 
        img = img / 255.0
        if len(img.shape) == 3:
            img = img[None]

        t2 = time_sync()
        dt[0] += t2 - t1

        pred = model(img, augment=False, visualize=False)[0]

        t3 = time_sync()
        dt[1] += t3 - t2

        pred = non_max_suppression(pred, 0.3, 0.45, None, 1000) # conf_thres, iou_thres, ?, max_def

        dt[2] += time_sync() - t3

        for i, det in enumerate(pred):
            seen += 1
            p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)
            #s += '%gx%g ' % img.shape[2:]  
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  
            imc = im0
            annotator = Annotator(im0, line_width=3, example=str(names))
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "

                for *xyxy, conf, cls in reversed(det): 
                    c = int(cls)  
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                    posx = int(xywh[0] * 640)
                    posy = int(xywh[1] * 480)
                    self.distance_cm = 0.1*self.callback_depthimage[posx, posy]
                    label = f'{names[c]} {conf:.2f} {self.distance_cm:.2f}cm'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    print(f'{s}{self.distance_cm:.2f}cm')

                #print(f'{s}Done. {1/(t3 - t2):.3f}fps')

        cv2.imshow('Detection', im0)
        cv2.waitKey(1)

def main():
    color_image_topic = '/camera/color/image_raw'
    depth_image_topic = '/camera/depth/image_rect_raw'

    detector = Detector(color_image_topic, depth_image_topic)
    rospy.spin()

if __name__ == "__main__": 
    set_logging()
    device = ''
    device = select_device(device)
    weights = '/home/jetson/catkin_ws/src/detection/scripts/best.pt'
    imgsz = 640
    model = attempt_load(weights, map_location=device)

    rospy.init_node('detection')
    main()
