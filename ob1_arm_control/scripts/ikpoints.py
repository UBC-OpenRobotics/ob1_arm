import random
from re import S
import time
from typing import Literal
import numpy as np
import pickle
from scipy.spatial import KDTree

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

    def get_nearest_joint_targets(self,pt1,num_pts=1):
        """
        @brief
        Given a point, returns a joint target to reach an area closest to that point
        
        @param pt1: numpy array for 3d point, eg. np.array([x,y,z])

        @param(optional, default=1) num_pts: number of nearest points to find joint targets for

        @return joint_target [j1,j2,j3,...] or list of joint targets 
        """

        if type(pt1) is not np.ndarray and  pt1.size != 3:
            print("parameter is not a valid point; needs np.array([floatx,floaty,floatz])")
            return None

        #tree search for closest point
        start = time.time()
        dist, index = self.kdtree.query(pt1, k=num_pts)
        print("search duration: %s seconds" %(time.time()-start))


        if type(dist) is np.ndarray:
            print("found %d closest points with avg distance %f cm" %(index.size, np.average(dist)*100))
            joints = []
            for i in index:
                joints.append(self.joint_targets[i].tolist())
            return joints
        else:
            print("found point with promimity %s" % dist)
            return self.joint_targets[index].tolist()

    def get_nearest_neighbour_pt(self,pt1):
        """
        get closest point not equal to this point

        @param pt1: numpy array for 3d point, eg. np.array([x,y,z])

        @return 3d point ndarray : np.array([x,y,z])
        """
        min_magnitude = -1
        index = -1
        for i in range(self._size):
            pt2 = self.points[i]
            dist = np.linalg.norm(pt1-pt2)
            if (dist < min_magnitude or min_magnitude < 0) and not np.array_equal(pt1,pt2):
                index = i
                min_magnitude = dist
        
        if index >= 0 and index < self._size:
            print("found point with promimity %s" % min_magnitude)
            return self.points[index]
        else:
            return None
    
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

        






