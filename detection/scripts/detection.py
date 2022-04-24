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
from pathlib import Path
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')


def LoadImg(img):
    img_size = 640
    cap = None
    path = None
    img0 = img
    img = letterbox(img0, new_shape=img_size)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img)
    return path, img, img0, cap

def run(image):
    dataset = LoadImg(image)

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
    img = img.float()  # uint8 to fp16/32
    img = img / 255.0  # 0 - 255 to 0.0 - 1.0
    if len(img.shape) == 3:
        img = img[None]  # expand for batch dim

    t2 = time_sync()
    dt[0] += t2 - t1

    pred = model(img, augment=False, visualize=False)[0]

    t3 = time_sync()
    dt[1] += t3 - t2

    pred = non_max_suppression(pred, 0.3, 0.45, None, 1000)

    dt[2] += time_sync() - t3

    for i, det in enumerate(pred):
        seen += 1
        p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)
        s += '%gx%g ' % img.shape[2:]  # print string
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        imc = im0
        annotator = Annotator(im0, line_width=3, example=str(names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in reversed(det): # Add bbox to image
                c = int(cls)  # integer class
                label = f'{names[c]} {conf:.2f}'
                annotator.box_label(xyxy, label, color=colors(c, True))

        # Print time (inference-only)
        print(f'{s}Done. ({t3 - t2:.3f}s)')

    cv2.imshow('Detection', im0)
    cv2.waitKey(1)

def image_callback(image):
    callback_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    callback_image = callback_image[...,::-1] #BGR to RGB
    with torch.no_grad():
        run(callback_image)

if __name__ == "__main__": 
    # Initialize
    set_logging()
    device = ''
    device = select_device(device)
    weights = '/home/jetson/catkin_ws/src/detection/scripts/best.pt'
    imgsz = 640

    model = attempt_load(weights, map_location=device)

    rospy.init_node('detection')
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback, queue_size=1, buff_size=52428800)

    rospy.spin()
