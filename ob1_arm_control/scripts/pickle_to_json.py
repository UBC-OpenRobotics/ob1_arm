#!/usr/bin/env python3.8
import glob
import json
import orjson
import rospkg
import pickle
from geometry_msgs.msg import Pose, PoseStamped
from rospy_msg_converter import convert_ros_message_to_dictionary

rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path('ob1_arm_control')

pickle_file_names = glob.glob(PACKAGE_PATH + '/data/*.pickle')

for pickle_file_name in pickle_file_names:
    data = None
    file_name = pickle_file_name.split('/')[-1].split('.')[0]
    json_file_name = PACKAGE_PATH + '/data/' + file_name + '.json'
    with open(pickle_file_name,"rb") as pf:
        data = pickle.load(pf)
    if data != None:
        for i in range(len(data)):
            data[i]["pose_stamped"] = convert_ros_message_to_dictionary(data[i]["pose_stamped"])
        with open(json_file_name,"w") as jf:
            print('Saved %s to %s' % (pickle_file_name,json_file_name))
            json.dump(data, jf)