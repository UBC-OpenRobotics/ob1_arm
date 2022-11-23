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
from geometry_msgs.msg import Pose, Point, Quaternion
import h5py
import tables
import multitables


#Author: Yousif El-Wishahy
#caclualtion functions for a database of inverse kinematics points
#use scipy's spacial kdtree to optimize search times

class IKPoints():
    """
    @brief psuedo inverse kinematics magics
    """
    #list of pose targets 
    pq_list:list = list()

    #list of vector3 (numpy array) points
    points:list = list()

    #list of joint targets
    joint_targets:list = list()

    _size:int 
    
    #kdtree for nearest point lookups
    position_kdtree:KDTree

    pose_kdtree:KDTree

    def __init__(self, file_path):
        file_type = file_path.split('.')[1]
        if file_type == 'h5':
            # ikpoints = tables.open_file(file_path, mode='r')
            # a = ikpoints.root.joint_data[:]
            # reader = multitables.Reader(filename=file_path, n_procs=6)
            # pq_dataset = reader.get_dataset(path='/pose_data')
            # stage = pq_dataset.create_stage(shape=pq_dataset.shape)
            # a = list(pq_dataset[:1000])
            ikpoints = h5py.File(file_path,mode='r')
        else:
            with open(file_path, "rb") as input_file:
                if file_type == 'pickle':
                    ikpoints = pickle.load(input_file)
                if file_type == 'json':
                    ikpoints = orjson.loads(input_file.read())

        print("============ Loaded ik points data file (%s)"%file_type)
        if file_type == 'h5':
            self._size = len(ikpoints["pose_data"])
            self.pq_list = ikpoints.get("pose_data")[:]
            self.points = ikpoints.get("point_data")[:]
            self.joint_targets = ikpoints.get("joiny_data")[:]
        else:
            print("============ Processing %d data objects on %d CPU Cores..." % (len(ikpoints),mp.cpu_count()))
            self._size = len(ikpoints)
            #populate np arrays
            _, self.joint_targets, self.points, self.pq_list = IKPoints.parallel_process(ikpoints)

        assert len(self.points) == self._size, 'points list does not match ikpoints size'
        assert len(self.joint_targets) == self._size, 'joint targets list does not match ikpoints size'
        assert len(self.pq_list) == self._size, 'position_quaternion list does not match ikpoints size'

        print("============ Loaded %s ik points" % (self._size))
        print('============ generating kd-trees...')
        self.position_kdtree = KDTree(self.points)
        self.pose_kdtree = KDTree(self.pq_list)

    @staticmethod
    def parallel_process(ikpoints:list, timeout=20):
        """
        Utilize multiple processor cores to load ikpoints faster

        @param ikpoints list as a loaded pickle or yaml
        
        @return tuple of lists (pose_targets, joint_targets, points, pq_list)
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
        
        def process(ikpoints_batch, queue, shared_counter):
            """
            Function to process ikpoints_batch
            Ideally run on multiple cpu cores in parallel
            """
            joint_targets = []
            pose_targets = []
            points = []
            pq_list = []
            for ikpoint in ikpoints_batch:
                if type(ikpoint["pose_stamped"]) is dict:
                    pose = convert_dictionary_to_ros_message('geometry_msgs/Pose', ikpoint["pose_stamped"]["pose"])
                else:
                    pose = ikpoint["pose_stamped"].pose
                p  = pose.position
                q = pose.orientation
                pq_list.append(np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w]))
                points.append(np.array([p.x, p.y, p.z]))
                pose_targets.append(pose)
                joint_targets.append(list(ikpoint["joint_target"]))
            queue.put((pose_targets, joint_targets, points, pq_list))
            queue.close()
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
        pq_list = []
        for _ in range(mp.cpu_count()):
            batch = queue.get()
            pose_targets+=batch[0]
            joint_targets+=batch[1]
            points+=batch[2]
            pq_list+=batch[3]
        return pose_targets, joint_targets, points, pq_list

    def get_nearest_points(self,pt1,n=1):
        """Queries kdtree and returns index of nearest point or sorted list (ascending) of indices of nearest points"""
        if type(pt1) is not np.ndarray and  pt1.size != 3:
            raise ValueError("parameter is not a valid point; needs np.array([floatx,floaty,floatz])")
        return self.position_kdtree.query(pt1, k=n)
    
    def get_nearest_poses(self,pose:Pose,n=1):
        """Queries kdtree and returns index of nearest pose or sorted list (ascending) of indices of nearest points"""
        if type(pose) is not Pose:
            raise ValueError("parameter is not a valid point; needs to be geometry_msgs/Pose)")
        p = pose.position
        q = pose.orientation
        pq = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        return self.pose_kdtree.query(pq, k=n)

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
        indices = self.position_kdtree.query_ball_point(pt1,dist+tolerance)
        out_indices = list()
        for i in indices:
            pt = self.points[i]
            d = np.linalg.norm(pt-pt1)
            if np.abs(dist-d) <= tolerance:
                out_indices.append(i)
        return out_indices

    def get_nearest_joint_targets(self,p,num_pts=1):
        """
        @brief
        Given a point or pose, returns a joint target to reach an area closest to that point or pose
        
        @param p: point np.array([x,y,z]) OR geometry_msgs/Pose
        @param(optional, default=1) num_pts: number of nearest points to find joint targets for

        @return joint_target [j1,j2,j3,...] or list of joint targets 
        """
        start = time.time()
        if type(p) is np.ndarray:
            dist, index = self.get_nearest_points(p,num_pts)
        elif type(p) is Pose:
            dist, index = self.get_nearest_poses(p,num_pts)
        else:
            raise ValueError("Input is not a pose or a position array, input type: %s" % type(p))
        search_dur = time.time() - start
        if type(dist) is np.ndarray:
            print("[IK Points] Found %d closest points with avg distance %.5f cm in %.5f seconds" %(index.size, np.average(dist)*100, search_dur))
            return [self.joint_targets[i].tolist() for i in index]
        else:
            print("[IK Points] Found point with promimity %.5f m in %.5f s" % (dist, search_dur))
            joints:list = self.joint_targets[index].tolist()
            return joints

    def get_dist_pose_targets(self, pt1, dist, tolerance = 0.01):
        """
        @brief Uses get_points_dist to find all pose targets float dist away (+/- tolerance) from numpy array vector pt1

        @param pt1 : float vector3 of type numpy.ndarray 
        @param dist: float distance
        @param tolerance: float tolerance
        
        @return list of poses matchting the distance +/- tolerance
        """
        indices = self.get_points_dist(pt1,dist, tolerance)
        return [Pose(Point(p[0], p[1], p[2]), Quaternion(p[3], p[4], p[5], p[6])) for p in [self.pq_list[i] for i in indices]]

    def get_dist_joint_targets(self, pt1, dist, tolerance = 0.01):
        """     
        @brief Uses get_points_dist to find all joint targets float dist away (+/- tolerance) from numpy array vector pt1
        Joint targets reach the points that match the search query.

        @param pt1 : float vector3 of type numpy.ndarray 
        @param dist: float distance
        @param tolerance: float tolerance
        
        @return list of joint targets
        """
        return [self.joint_targets[i].tolist() for i in self.get_points_dist(pt1, dist, tolerance)]
    
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
        indices = self.get_points_dist(pt1, dist, tolerance)
        joints = [self.joint_targets[i].tolist() for i in indices]
        poses = [Pose(Point(p[0], p[1], p[2]), Quaternion(p[3], p[4], p[5], p[6])) for p in [self.pq_list[i] for i in indices]]
        assert len(poses) == len(joints), "pose and joints lists are not the same length"
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
        dist,_= self.position_kdtree.query(pt)
        return dist <= tolerance

        






