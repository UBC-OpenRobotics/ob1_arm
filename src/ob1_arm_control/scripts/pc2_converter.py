#!/usr/bin/env python
from configparser import MAX_INTERPOLATION_DEPTH
import mimetypes
import rospy
from pcl import PointCloud
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import tf
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import random
import threading

def callback(data):
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    p = pcl.PointCloud(np.array(points, dtype=np.float32))

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/placeholder_camera/depth/points", PointCloud2, callback)
rospy.spin()

#inspiration
#https://github.com/PacktPublishing/ROS-Robotics-By-Example/blob/master/Chapter_9_code/crazyflie_autonomous/scripts/detect_crazyflie.py
#https://github.com/Choitek/mmmros-docs/blob/master/tutorials/kinect.md

#Class to detect oject location and dimensions in camera ref frame
#does this by overlaying pixel bounding box (from cv team) with kinect depth image
#kinect depth data : mm 
class ObjectDetector():
    #todo update these topic names to correct names
    depth_img_topic = "/placeholder_camera//depth/image_raw" 
    color_img_topic = "/placeholder_camera/color/image_raw"

    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)
        self.pub_tf = tf.TransformBroadcaster()
        self.depthimg_sub = rospy.Subscriber(self.depth_img_topic, Image, self.cb_depth_img)
        self.rate = rospy.Rate(50.0)
        self.cv_bridge = CvBridge()

        #todo: where does this class get the bounding box data
        #is it a subscriber?
        self.bbox = [0,0,0,0] #x1,y1 (top left),x2,y2 (bottom right) pixel coords 
        self.depth_img = Image()
        self.empty = True
        self.object_coords = [0,0,0] #x,y,depth (z)

    def cb_depth_img(self,img_msg):
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            self.empty = True
        except CvBridgeError as e:
            print(e)

    def process_depth_img(self):
        if not self.empty:
            overlay_pts = self.depth_img[self.bbox[0]:self.bbox[2],self.bbox[1]:self.bbox[3]]
            min_depth = overlay_pts.min()
            max_depth = overlay_pts.max()
            mid_depth = (max_depth-min_depth)/2
            self.object_coords[2] = mid_depth
            self.empty = True

    #not sure if this is correct
    def update_bb_data(self,bounding_box):
        self.bbox = bounding_box
        #assume object to be located at centre of bounding box
        self.object_coords = [
            self.bbox[2] - self.bbox[0], 
            self.bbox[3] - self.bbox[1], 0]

