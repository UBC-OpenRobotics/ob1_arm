#! /usr/bin/env python
from __future__ import print_function
import time
import rospy
from copy import deepcopy
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import pickle
from arm_commander import ArmCommander
import rospkg
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path('ob1_arm_control')
DATA_FILE_PATH = PACKAGE_PATH+"/data/marker_data.pickle"

def publish_markers():
    """
    Publish marker ros message contiously such that the marker topic can be viewed in RVIZ
    """

    #init
    rospy.init_node("reachability_visualizer", anonymous=True)
    pub = rospy.Publisher('/reachability_visualizier', Marker, queue_size=10)

    #read data
    marker:Marker = None
    with open(DATA_FILE_PATH, "rb") as input_file:
        marker = pickle.load(input_file)

    if marker != None:
        points = Marker()
        points.header.frame_id = marker.header.frame_id
        points.header.stamp = marker.header.stamp
        points.ns = marker.ns
        points.action = 0
        points.id = marker.id
        points.type = 1
        points.scale = Vector3(.1,.1,.1)
        points.color = ColorRGBA(r=0, g=165, b=8, a=0.8)

        for i in range(len(marker.points)):
            point = marker.points[i]
            color = marker.colors[i]
            points.points.append(point)
            points.colors.append(color)

        while not rospy.is_shutdown():
            pub.publish(points)
            time.sleep(5)

def plot_markers_matplotlib():
    marker:Marker = None
    with open(DATA_FILE_PATH, "rb") as input_file:
        marker = pickle.load(input_file)

    if marker != None:
        x=[]
        y=[]
        z=[]
        for point in marker.points:
            x.append(point.x)
            y.append(point.y)
            z.append(point.z)

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter(x, y, z)
        plt.show()
    else:
        print("marker data is NONE. Check data file.")


if __name__ == '__main__':
    # publish_markers()
    plot_markers_matplotlib()