#! /usr/bin/env python

import roslib
import rospy
import tf
from detection.msg import Object
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from visualization_msgs.msg import Marker

import argparse
import math
import numpy as np

class Visualization:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()

        self.camerapose = [0, 0, 0] #realsense pose for transform

        self.subslampose = rospy.Subscriber('/slam_out_pose', PoseStamped, self.SlamPoseCallback, queue_size=1)
        #self.subobjpoint = rospy.Subscriber('/obj_out_point', Point, self.ObjPointCallback, queue_size=1)

        self.puboptmarker = rospy.Publisher("/opt_marker", Marker, queue_size=1)

        self.transform()

    def SlamPoseCallback(self, slamout):
        self.slam_position = [slamout.pose.position.x, slamout.pose.position.y, slamout.pose.position.z]
        self.slam_orientation = [slamout.pose.orientation.x, slamout.pose.orientation.y, slamout.pose.orientation.z, slamout.pose.orientation.w]
        self.br.sendTransform((self.position[0] + self.camerapose[0], self.position[1] + self.camerapose[1], self.position[2] + self.camerapose[2]), 
                              (self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]), 
                               rospy.Time.now(), "realsense", "map")

    def ObjectCallback(self.

 #   def ObjPointCallback(self, objout):
 #       self.obj_position = [objout.x, objout.y, objout.z]

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
        
        pose.pose.orientation.x = self.slam_orientation[0]
        pose.pose.orientation.y = self.slam_orientation[1]
        pose.pose.orientation.z = self.slam_orientation[2]
        pose.pose.orientation.w = self.slam_orientation[3]

        self.g_coord = self.ls.transformPose("map", pose)
       
    def transform(self):
        self.init_opt_marker()
        self.draw_opt_marker(float(self.det_points[i, 1:][0]), float(self.det_points[i, 1:][1]))


        self.setpose(d_point_ros[0], d_point_ros[1])

        posgx = self.g_coord.pose.position.x
        posgy = self.g_coord.pose.position.y

        self.puboptmarker.publish(self.opt_marker)


def main():
    visualization = Visualization()

if __name__ == "__main__": 
    rospy.init_node('Visualization')
    main()
