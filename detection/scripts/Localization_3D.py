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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from visualization_msgs.msg import Marker

ROOT = os.getcwd()
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

class Localization:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()

        self.pose_ready = False

        ##Realsense Position for Transform
        self.camerapose = [0, 0, 0]

        self.det_points = np.array([["class_id", "class_name", "x", "y", "z", "sumx", "sumy", "sumz", "num"]])

        self.subpose = rospy.Subscriber('/integrated_to_init', Odometry, self.poseCallback, queue_size=1)
        self.subobject = rospy.Subscriber('/det_out_obj', Object, self.objectCallback, queue_size=1)

        self.puboptmarker = rospy.Publisher("/marker", Marker, queue_size=1)
        self.pubobjoutput = rospy.Publisher("/obj_output", Object, queue_size=1)

    def poseCallback(self, slamout):
        self.position = [slamout.pose.pose.position.z, slamout.pose.pose.position.x, slamout.pose.pose.position.y]
        self.orientation = [slamout.pose.pose.orientation.z, slamout.pose.pose.orientation.x, slamout.pose.pose.orientation.y, slamout.pose.pose.orientation.w]
        self.br.sendTransform((self.position[0] + self.camerapose[0], self.position[1] + self.camerapose[1], self.position[2] + self.camerapose[2]), 
                              (self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]), 
                               rospy.Time.now(), "realsense", "map")
        self.pose_ready = True

    def objectCallback(self, object):
        self.posx = object.x
        self.posy = object.y
        self.posz = object.z
        self.class_name = object.class_name
        if(self.pose_ready == True):
            self.localization()
        self.pose_ready == False

    def draw_marker(self, n_class, marker_id, x, y, z):
        ##For Object Marker
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.ns = n_class
        marker.id = int(marker_id)
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.color = ColorRGBA(0, 0, 1, 1)
        marker.scale = Vector3(0.5, 0.5, 0.5)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 0

        ##For Text Marker
        marker_txt = Marker()
        marker_txt.ns = "text"
        marker_txt.action = Marker.ADD
        marker_txt.id = int(marker_id)
        marker_txt.header.frame_id = "/map"
        marker_txt.type = Marker.TEXT_VIEW_FACING
        marker_txt.text = n_class
        marker_txt.color = ColorRGBA(1, 1, 1, 1)
        marker_txt.scale.z = 0.5
        marker_txt.pose.position = Point(x, y, z + 0.6)

        self.puboptmarker.publish(marker)
        self.puboptmarker.publish(marker_txt)

    def setpose(self, px, py, pz):
        pose = PoseStamped()
        
        pose.header.frame_id = "realsense"

        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        
        pose.pose.orientation.x = self.orientation[0]
        pose.pose.orientation.y = self.orientation[1]
        pose.pose.orientation.z = self.orientation[2]
        pose.pose.orientation.w = self.orientation[3]

        self.g_coord = self.ls.transformPose("map", pose)
       
    def localization(self):
        if self.det_points.shape[0] > 1:
            for i in range(1, self.det_points.shape[0]):
                self.draw_marker(self.det_points[i, 1], self.det_points[i, 0], float(self.det_points[i, 1:][1]), float(self.det_points[i, 1:][2]), float(self.det_points[i, 1:][3]))
        d_dist = math.sqrt(pow(self.posx, 2) + pow(self.posy, 2) + pow(self.posz, 2))

        if d_dist > 0.05 and d_dist < 4:
            self.setpose(self.posx, self.posy, self.posz)

            posgx = self.g_coord.pose.position.x
            posgy = self.g_coord.pose.position.y
            posgz = self.g_coord.pose.position.z

            w_class = np.where(self.det_points == self.class_name)[0] #Find class
            n_class = w_class.shape[0] #Number of Known class

            knn_score = 0
            knn_rst = 1

            dist_threshold = 0.6

            if n_class > 0:
                for i in w_class:
                    w_x = abs(posgx - float(self.det_points[i, 1:][1])) #x
                    w_y = abs(posgy - float(self.det_points[i, 1:][2])) #y
                    w_z = abs(posgz - float(self.det_points[i, 1:][3])) #z
                    dist = math.sqrt(pow(w_x, 2) + pow(w_y, 2) + pow(w_z, 2))
                    if dist >= dist_threshold:
                        knn_score += 1
                    elif dist < dist_threshold and dist >= 0.08:
                        self.det_points[i, 1:][7] = int(self.det_points[i, 1:][7]) + 1
                        self.det_points[i, 1:][4] = float(self.det_points[i, 1:][4]) + float(posgx) #sumx
                        self.det_points[i, 1:][5] = float(self.det_points[i, 1:][5]) + float(posgy) #sumy
                        self.det_points[i, 1:][6] = float(self.det_points[i, 1:][6]) + float(posgz) #sumz
                        self.det_points[i, 1:][1] = float(self.det_points[i, 1:][4]) / float(self.det_points[i, 1:][7]) #avgx
                        self.det_points[i, 1:][2] = float(self.det_points[i, 1:][5]) / float(self.det_points[i, 1:][7]) #avgy
                        self.det_points[i, 1:][3] = float(self.det_points[i, 1:][6]) / float(self.det_points[i, 1:][7]) #avgz
                        knn_rst = 0
                    elif dist < 0.1:
                        knn_rst = 0
                if knn_score * knn_rst != 0:
                    self.det_points = np.append(self.det_points, [[self.det_points.shape[0] - 1, self.class_name, posgx, posgy, posgz, posgx, posgy, posgz, 1]], axis=0) #add det_points
            elif n_class == 0:
                self.det_points = np.append(self.det_points, [[self.det_points.shape[0] - 1, self.class_name, posgx, posgy, posgz, posgx, posgy, posgz, 1]], axis=0) #add det_points
        #print(self.det_points[:, :5])

def main():
    localization = Localization()
    rospy.spin()

if __name__ == "__main__": 
    rospy.init_node('localization')
    main()
