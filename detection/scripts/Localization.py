#! /usr/bin/env python

import roslib
import rospy
import tf
import os
import os.path as osp
import math
import sys
import numpy as np
from detection.msg import Object
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from visualization_msgs.msg import Marker

ROOT = os.getcwd()
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

#from pathlib import Path
#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

class Localization:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()

        self.camerapose = [0, 0, 0] #realsense pose for transform

        self.det_points = np.array([["class_id", "class_name", "x", "y", "sumx", "sumy", "num"]])

        self.subpose = rospy.Subscriber('/slam_out_pose', PoseStamped, self.poseCallback, queue_size=1)
        self.subobject = rospy.Subscriber('/det_out_obj', Object, self.objectCallback, queue_size=1)

        self.puboptmarker = rospy.Publisher("/marker", Marker, queue_size=1)
        self.pubobjoutput = rospy.Publisher("/obj_output", Object, queue_size=1)

        #self.file = open(osp.join(ROOT, 'catkin_ws/src/detection/scripts/output/output.txt'), 'w')

    def poseCallback(self, slamout):
        self.position = [slamout.pose.position.x, slamout.pose.position.y, slamout.pose.position.z]
        self.orientation = [slamout.pose.orientation.x, slamout.pose.orientation.y, slamout.pose.orientation.z, slamout.pose.orientation.w]
        self.br.sendTransform((self.position[0] + self.camerapose[0], self.position[1] + self.camerapose[1], self.position[2] + self.camerapose[2]), 
                              (self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]), 
                               rospy.Time.now(), "realsense", "map")

    def objectCallback(self, object):
        self.posx = object.x
        self.posy = object.y
        self.posz = object.z
        self.class_name = object.class_name
        self.localization()

    def init_marker(self):
        self.marker = Marker()
        self.marker.header.frame_id = "/map"
        self.marker.ns = "marker"
        self.marker.id = 1
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.color = ColorRGBA(0, 0, 1, 1)
        self.marker.scale = Vector3(0.2, 0.2, 0)

    def draw_marker(self, x, y):
        self.marker.points.append(Point(x, y, 0))

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
       
    def localization(self):
        self.init_marker()

        if self.det_points.shape[0] > 1:
            for i in range(1, self.det_points.shape[0]):
                self.draw_marker(float(self.det_points[i, 1:][1]), float(self.det_points[i, 1:][2]))
        print(self.det_points)
        print('=========================' + '\n')

        d_dist = self.posx + self.posy + self.posz

        if d_dist != 0 and self.posx < 10:
            #self.file.write('{self.class_name} : {self.posx}, {self.posy}, {self.posz}' + '\n')
            self.setpose(self.posx, self.posy)

            posgx = self.g_coord.pose.position.x
            posgy = self.g_coord.pose.position.y

            w_class = np.where(self.det_points == self.class_name)[0] #Find class
            n_class = w_class.shape[0] #Number of Known class

            knn_score = 0
            knn_rst = 1

            dist_threshold = 0.4

            if n_class > 0:
                for i in w_class:
                    w_x = abs(posgx - float(self.det_points[i, 1:][1])) # x
                    w_y = abs(posgy - float(self.det_points[i, 1:][2])) # y
                    dist = math.sqrt(pow(w_x, 2) + pow(w_y, 2))
                    if dist >= dist_threshold:
                        knn_score += 1
                    elif dist < dist_threshold and dist >= 0.05:
                        self.det_points[i, 1:][5] = int(self.det_points[i, 1:][5]) + 1
                        self.det_points[i, 1:][3] = float(self.det_points[i, 1:][3]) + float(posgx)
                        self.det_points[i, 1:][4] = float(self.det_points[i, 1:][4]) + float(posgy)
                        self.det_points[i, 1:][1] = float(self.det_points[i, 1:][3]) / float(self.det_points[i, 1:][5])
                        self.det_points[i, 1:][2] = float(self.det_points[i, 1:][4]) / float(self.det_points[i, 1:][5])
                        knn_rst = 0
                    elif dist < 0.1:
                        knn_rst = 0
                if knn_score * knn_rst != 0:
                    self.det_points = np.append(self.det_points, [[self.det_points.shape[0] - 1, self.class_name, posgx, posgy, posgx, posgy, 1]], axis=0) #add det_points
            elif n_class == 0:
                self.det_points = np.append(self.det_points, [[self.det_points.shape[0] - 1, self.class_name, posgx, posgy, posgx, posgy, 1]], axis=0) #add det_points

        self.puboptmarker.publish(self.marker)
        #self.file.write('====================================' + '\n')


def main():
    localization = Localization()
    rospy.spin()

if __name__ == "__main__": 
    rospy.init_node('localization')
    main()
