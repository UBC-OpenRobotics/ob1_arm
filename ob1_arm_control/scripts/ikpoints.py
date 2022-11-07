import random
from re import S
import time
from typing import Literal
import numpy as np
import pickle
from scipy.spatial import KDTree, distance


#Author: Yousif El-Wishahy
#caclualtion functions for a database of inverse kinematics points
#use scipy's spacial kdtree to optimize search times

class IKPoints():
    """
    @brief
    format of ikpoint:
    ikpoint = {"pose_stamped":eef_pose_stamped,"joint_target":joints}
    pose_stamped is a geometry_msgs PoseStamped object
    joint_target is a list of floats
    """
    #list of ikpoints
    ikpoints:list = []

    #list poses
    pose_targets:list = list()

    #np array of 3d points
    points:np.ndarray = np.array([])

    #np array of joint targets to reach corresponding point
    joint_targets:np.ndarray = np.array([])

    _size:int 
    
    #kdtree for nearest point lookups
    kdtree:KDTree

    def __init__(self, file_path):
        try:
            with open(file_path, "rb") as input_file:
                self.ikpoints = pickle.load(input_file)
                print("============ Loaded %s ik points" % (len(self.ikpoints)))
        except Exception as errmsg:
            print(errmsg)
            print("Failed to load ikpoints file.")

        #populate np arrays
        points = []
        joint_targets = []
        for ikpoint in self.ikpoints:
            joint_targets.append(ikpoint["joint_target"])
            self.pose_targets.append(ikpoint["pose_stamped"].pose)
            p  = ikpoint["pose_stamped"].pose.position
            point = np.array([p.x,p.y,p.z])
            points.append(point)
        if len(points) == len(joint_targets):
            self._size = len(points)
            self.points = np.array(points)
            self.joint_targets = np.array(joint_targets)
            self.kdtree = KDTree(self.points)
            print(self.points.shape,self.points.size)
        else:
            print("POINTS AND JOINT TARGET ARRAYS NOT THE SAME LENGTH.")

    def get_nearest_points(self,pt1,num_pts=1):
        """Queries kdtree and returns index of nearest point or sorted list (ascending) of indices of nearest points"""
        if type(pt1) is not np.ndarray and  pt1.size != 3:
            raise ValueError("parameter is not a valid point; needs np.array([floatx,floaty,floatz])")
        return self.kdtree.query(pt1, k=num_pts)

    def get_points_dist(self,pt1,dist,tolerance=0.01):
        """Return a list of indices of points a certain distance away from pt1 with a specified tolerance"""
        if type(pt1) is not np.ndarray and  pt1.size != 3:
            raise ValueError("parameter is not a valid point; needs np.array([floatx,floaty,floatz])")
        indices = self.kdtree.query_ball_point(pt1,dist)
        out_indices = list()
        for i in indices:
            pt = self.points[i]
            d = np.linalg.norm(pt-pt1)
            if np.abs(dist-d) <= tolerance:
                out_indices.append(i)
        return out_indices

    def get_nearest_joint_targets(self,pt1,num_pts=1):
        """
        @brief
        Given a point, returns a joint target to reach an area closest to that point
        
        @param pt1: numpy array for 3d point, eg. np.array([x,y,z])

        @param(optional, default=1) num_pts: number of nearest points to find joint targets for

        @return joint_target [j1,j2,j3,...] or list of joint targets 
        """
        dist, index = self.get_nearest_points(pt1,num_pts)
        if type(dist) is np.ndarray:
            print("found %d closest points with avg distance %f cm" %(index.size, np.average(dist)*100))
            joints = []
            for i in index:
                joints.append(self.joint_targets[i].tolist())
            return joints
        else:
            print("found point with promimity %s" % dist)
            joints:list = self.joint_targets[index].tolist()
            return joints
    
    def get_nearest_pose_targets(self,pt1,num_pts=1):
        """
        @brief
        Given a point, returns a pose target to reach an area closest to that point
        
        @param pt1: numpy array for 3d point, eg. np.array([x,y,z])

        @param(optional, default=1) num_pts: number of nearest points to find joint targets for

        @return joint_target [j1,j2,j3,...] or list of joint targets 
        """
        dist, index = self.get_nearest_points(pt1,num_pts)
        if type(dist) is np.ndarray:
            print("found %d closest points with avg distance %f cm" %(index.size, np.average(dist)*100))
            poses = []
            for i in index:
                poses.append(self.pose_targets[i])
            return poses
        else:
            print("found point with promimity %s" % dist)
            return self.pose_targets[i]

    def get_dist_pose_targets(self, pt1, dist):
        poses = []
        indices = self.get_points_dist(pt1,dist)
        if len(indices) > 0:
            for i in indices:
                poses.append(self.pose_targets[i])
        return poses

    def get_dist_joint_targets(self, pt1, dist):
        joints = []
        indices = self.get_points_dist(pt1,dist)
        if len(indices) > 0:
            for i in indices:
                joints.append(self.joint_targets[i].tolist())
        return joints

    def get_rand_point(self):
        index = random.randint(0, self._size)
        return self.points[index].tolist()

    def in_range(self,pt,tolerance):
        """
        @pt1 returns true if pt is within tolerance range of ikpoint dataset

        @param pt: np.ndarray [x,y,z]

        @return bool
        """

        dist,_= self.kdtree.query(pt)

        return dist <= tolerance

        






