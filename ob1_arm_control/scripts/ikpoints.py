import random
from re import S
import time
import numpy as np
import pickle

#Author: Yousif El-Wishahy
#caclualtion functions for a database of inverse kinematics points
#very unoptimized

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
            print(self.points.shape,self.points.size)
        else:
            print("POINTS AND JOINT TARGET ARRAYS NOT THE SAME LENGTH.")

    def get_nearest_joint_target(self,pt1):
        """
        @brief
        Given a point, returns a joint target to reach an area closest to that point
        
        @param point: numpy array for 3d point, eg. np.array([x,y,z])
        """

        if type(pt1) is not np.ndarray and  pt1.size != 3:
            print("parameter is not a valid point; needs np.array([floatx,floaty,floatz])")
            return None

        #perform a linear search to minimize vector magnitude
        #todo: look into faster ways of doing this
        index = -1
        min_magnitude = -1
        start = time.time()
        for i in range(self._size):
            pt2 = self.points[i]
            dist = np.linalg.norm(pt1-pt2)
            if dist < min_magnitude or min_magnitude < 0:
                index = i
                min_magnitude = dist
        print("search duration: %s seconds" %(time.time()-start))

        if index >= 0 and index < self._size:
            print("found point with promimity %s" % min_magnitude)
            return self.joint_targets[index].tolist()
        else:
            print("Could not find closest point and joint target")
            return None
    
    def get_rand_point(self):
        index = random.randint(0, self._size)
        return self.points[index].tolist()
        






