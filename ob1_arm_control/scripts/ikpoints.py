import random
import json
import orjson
import time
from typing import Literal
import numpy as np
import pickle
from scipy.spatial import KDTree, distance
import _compat_pickle
from rospy_msg_converter import convert_dictionary_to_ros_message
import multiprocessing as mp
from threading import Thread


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
    #list of pose targets 
    pose_targets:list = list()

    #list of vector3 (numpy array) points
    points:list = list()

    #list of joint targets
    joint_targets:list = list()

    _size:int 
    
    #kdtree for nearest point lookups
    kdtree:KDTree

    def __init__(self, file_path):
        with open(file_path, "rb") as input_file:
            if file_path.split('.')[1] == 'pickle':
                ikpoints = pickle.load(input_file)
            if file_path.split('.')[1] == 'json':
                ikpoints = orjson.loads(input_file.read())
        print("============ Loaded ik points data file")
        print("============ Processing %d data objects on %d CPU Cores..." % (len(ikpoints),mp.cpu_count()))

        #populate np arrays
        self.pose_targets, self.joint_targets, self.points = IKPoints.parallel_process(ikpoints)

        if len(self.points) == len(self.joint_targets):
            self._size = len(self.points)
            print('generating kd-tree...')
            self.kdtree = KDTree(self.points)
        else:
            print("POINTS AND JOINT TARGET ARRAYS NOT THE SAME LENGTH.")
        print("============ Loaded %s ik points" % (self._size))

    @staticmethod
    def parallel_process(ikpoints:list, timeout=20):
        """
        Utilize multiple processor cores to load ikpoints faster

        @param ikpoints list as a loaded pickle or yaml
        
        @return tuple of lists (pose_targets, joint_targets, points)
        """

        def split_ikpoints(ikpoints_all, n_split:int):
            size = len(ikpoints_all)
            seg = int(size/n_split)
            split_ikpoints = []
            for i in range(n_split):
                start = i*seg
                end = (i+1)*seg
                if start >= size:
                    break
                if end >= size:
                    end = size - 1
                split_ikpoints.append(ikpoints_all[start:end])
            print("CPU Cores: %d, Batch Size: %d"  %(len(split_ikpoints), len(split_ikpoints[0])))
            return split_ikpoints
        
        def process(ikpoints_batch, q, shared_counter):
            """
            Function to process ikpoints_batch
            Ideally run on multiple cpu cores in parallel
            """
            joint_targets = []
            pose_targets = []
            points = []
            for ikpoint in ikpoints_batch:
                joint_targets.append(list(ikpoint["joint_target"]))
                if type(ikpoint["pose_stamped"]) is dict:
                    pose_targets.append(convert_dictionary_to_ros_message('geometry_msgs/Pose', ikpoint["pose_stamped"]["pose"]))
                else:
                    pose_targets.append(ikpoint["pose_stamped"].pose)
                p  = ikpoint["pose_stamped"]
                if type(p) is dict:
                    p = p["pose"]["position"]
                    point = np.array([p['x'],p['y'],p['z']])
                else:
                    p = p.pose.position
                    point = np.array([p.x,p.y,p.z])
                points.append(point)
            q.put((pose_targets, joint_targets, points))
            q.close()
            shared_counter.value+=1
            return

        ikpoints_batch_list = split_ikpoints(ikpoints, mp.cpu_count())
        procs = []
        queue = mp.Queue()
        shared_counter = mp.Value('i', 0)

        #start the processes
        for ikpoints_batch in ikpoints_batch_list:
            proc = mp.Process(target=process, args=(ikpoints_batch,queue,shared_counter,))
            procs.append(proc)
            proc.start()

        #wait for the processes to finish (indicated by shared counter)
        start = time.time()
        while time.time() < start + timeout:
            if shared_counter.value >= mp.cpu_count():
                break
        
        #terminate all child processes
        for proc in procs:
            proc.terminate()
            print('ik_points_process: child process terminated successfully')
        
        #combine data
        joint_targets = []
        pose_targets = []
        points = []
        for _ in range(mp.cpu_count()):
            batch = queue.get()
            pose_targets+=batch[0]
            joint_targets+=batch[1]
            points+=batch[2]
        return pose_targets, joint_targets, points

    def get_nearest_points(self,pt1,num_pts=1):
        """Queries kdtree and returns index of nearest point or sorted list (ascending) of indices of nearest points"""
        if type(pt1) is not np.ndarray and  pt1.size != 3:
            raise ValueError("parameter is not a valid point; needs np.array([floatx,floaty,floatz])")
        return self.kdtree.query(pt1, k=num_pts)

    def get_points_dist(self,pt1,dist,tolerance):
        """
        @brief Return a list of indices of points a certain distance away from pt1 with a specified tolerance

        @param pt1 : float vector3 of type numpy.ndarray 
        @param dist: float distance
        @param tolerance: float tolerance
        
        @return list of indices corresponding to ikpoints that match the distance
        """
        if type(pt1) is not np.ndarray and  pt1.size != 3:
            raise ValueError("parameter is not a valid point; needs np.array([floatx,floaty,floatz])")
        indices = self.kdtree.query_ball_point(pt1,dist+tolerance)
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
        start = time.time()
        dist, index = self.get_nearest_points(pt1,num_pts)
        search_dur = time.time() - start
        if type(dist) is np.ndarray:
            print("[IK Points] Found %d closest points with avg distance %.5f cm in %.5f seconds" %(index.size, np.average(dist)*100, search_dur))
            joints = []
            for i in index:
                joints.append(self.joint_targets[i])
            return joints
        else:
            print("[IK Points] Found point with promimity %.5f m in %.5f s" % (dist, search_dur))
            joints:list = self.joint_targets[index]
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

    def get_dist_pose_targets(self, pt1, dist, tolerance = 0.01):
        """
        @brief Uses get_points_dist to find all pose targets float dist away (+/- tolerance) from numpy array vector pt1

        @param pt1 : float vector3 of type numpy.ndarray 
        @param dist: float distance
        @param tolerance: float tolerance
        
        @return list of poses matchting the distance +/- tolerance
        """
        poses = []
        indices = self.get_points_dist(pt1,dist, tolerance)
        if len(indices) > 0:
            for i in indices:
                poses.append(self.pose_targets[i])
        return poses

    def get_dist_joint_targets(self, pt1, dist, tolerance = 0.01):
        """
        @brief Uses get_points_dist to find all joint targets float dist away (+/- tolerance) from numpy array vector pt1
        Joint targets reach the points that match the search query.

        @param pt1 : float vector3 of type numpy.ndarray 
        @param dist: float distance
        @param tolerance: float tolerance
        
        @return list of joint targets
        """
        joints = []
        indices = self.get_points_dist(pt1, dist, tolerance)
        if len(indices) > 0:
            for i in indices:
                joints.append(self.joint_targets[i])
        return joints
    
    def get_dist_targets(self, pt1, dist, tolerance = 0.01):
        """
        @brief Returns both search results of get_dist_pose_targets and get_dist_joint_targets

        Returns 2 lists, each pose target in the first list has a 
        correspodning joint target at the same index in the second list.

        @param pt1 : float vector3 of type numpy.ndarray 
        @param dist: float distance
        @param tolerance: float tolerance

        @return tuple (list of pose targets, list of joint targets)
        """
        joints = []
        poses = []
        indices = self.get_points_dist(pt1, dist, tolerance)
        if len(indices) > 0:
            for i in indices:
                joints.append(self.joint_targets[i])
                poses.append(self.pose_targets[i])
        return poses, joints

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
        print(dist, _)

        return dist <= tolerance

        






