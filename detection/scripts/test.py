#! /usr/bin/env python3

import roslib
import rospy
import tf
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from visualization_msgs.msg import Marker

import argparse
import os
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
from cv_bridge import CvBridge, CvBridgeError

from pathlib import Path
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

class Detector:
    def __init__(self):
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)

        self.br = tf.TransformBroadcaster()

        self.camerapose = [0, 0, 0]

        #self.frame = 0

        self.subpose = rospy.Subscriber('/slam_out_pose', PoseStamped, self.poseCallback, queue_size=1)
#        self.pubpose = rospy.Publisher("poseout", PoseStamped, queue_size=1)
        self.pubresult = rospy.Publisher('/detection_result_image', Image, queue_size=1)
        self.pubmarker = rospy.Publisher("marker", Marker, queue_size=1)

        try:
            while True:
                with torch.no_grad():
                    self.initmarker()
                    self.detection()
                    key = cv2.waitKey(1)
                    if key == ord("q"):
                        break
        finally:
            self.pipeline.stop()

#    def transform(self): #TF
#        tf = PoseStamped()
#        
#        tf.header.stamp = rospy.Time.now()
#        tf.header.frame_id = "/camera_test"
#
#        tf.pose.position.x = self.position[0] + self.camerapose[0]
#        tf.pose.position.y = self.position[1] + self.camerapose[1]
#        tf.pose.position.z = self.position[2] + self.camerapose[2]
#        
#        tf.pose.orientation.x = self.orientation[0]
#        tf.pose.orientation.y = self.orientation[1]
#       tf.pose.orientation.z = self.orientation[2]
#        tf.pose.orientation.w = self.orientation[3]

#        self.pubpose.publish(tf)

    def poseCallback(self, slamout):
        self.position = [slamout.pose.position.x, slamout.pose.position.y, slamout.pose.position.z]
        self.orientation = [slamout.pose.orientation.x, slamout.pose.orientation.y, slamout.pose.orientation.z, slamout.pose.orientation.w]
        self.br.sendTransform((self.position[0], self.position[1], self.position[2]), 
                              (self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]), 
                               rospy.Time.now(), "camera_test", "map")
#        self.transform()

    def initmarker(self):
        self.det_marker = Marker()

        self.det_marker.header.frame_id = "/camera_test"
        self.det_marker.ns = "det"
        self.det_marker.id = 1
        self.det_marker.type = Marker.POINTS
        self.det_marker.action = Marker.ADD
        self.det_marker.color = ColorRGBA(1, 0, 0, 1)
        self.det_marker.scale = Vector3(0.2, 0.2, 0)

    def drawmarker(self, x, y, z):
        self.det_marker.points.append(Point(x, y, z))

    def Loadimage(self, img):
        cap = None
        path = None
        img0 = img
        img = letterbox(img0, new_shape=640)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        return path, img, img0, cap

    def detection(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

        color_image = np.asanyarray(color_frame.get_data())

        dataset = self.Loadimage(color_image)

        path = dataset[0]
        img = dataset[1]
        im0s = dataset[2]
        vid_cap = dataset[3]
    
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
            p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  
            imc = im0
            annotator = Annotator(im0, line_width=3, example=str(names))
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()
                    #s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "

                for *xyxy, conf, cls in reversed(det): 
                    c = int(cls)
  
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                    posx = int(xywh[0] * 640)
                    posy = int(xywh[1] * 480)
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, (posx,posy), depth_frame.get_distance(posx,posy))
                    depth_point_ros = [round(depth_point[2], 4), -round(depth_point[0], 4), -round(depth_point[1], 4)]

                    label = f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    cv2.putText(im0, str(depth_point_ros), (posx, posy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                    print(f'{names[c]} : {depth_point_ros}')

                    if depth_point_ros:
                        self.drawmarker(depth_point_ros[0], depth_point_ros[1], depth_point_ros[2])

        self.pubmarker.publish(self.det_marker)
        cv2.imshow('Detection', im0)
        #savepath = '/home/jetson/Desktop/test/' #Save Image
        #savepath += f"'{self.frame}'.jpg" 
        #cv2.imwrite(savepath, im0)
        #self.frame = self.frame + 1
        cv2.waitKey(1)
        self.pubresult.publish(self.bridge.cv2_to_imgmsg(im0, encoding="bgr8"))

def main():
    detector = Detector()
    rospy.spin()

if __name__ == "__main__": 
    set_logging()
    device = ''
    device = select_device(device)
    weights = '/home/jetson/catkin_ws/src/detection/scripts/yolov5s.pt'
    imgsz = 640
    model = attempt_load(weights, map_location=device)

    rospy.init_node('detection')
    main()
