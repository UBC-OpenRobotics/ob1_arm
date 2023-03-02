function [data, info] = iKPointsServiceResponse
%IKPointsService gives an empty data for ob1_arm_control/IKPointsServiceResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ob1_arm_control/IKPointsServiceResponse';
[data.PoseTargets, info.PoseTargets] = ros.internal.ros.messages.geometry_msgs.pose;
info.PoseTargets.MLdataType = 'struct';
info.PoseTargets.MaxLen = NaN;
info.PoseTargets.MinLen = 0;
data.PoseTargets = data.PoseTargets([],1);
[data.JointTargets, info.JointTargets] = ros.internal.ros.custommessages.ob1_arm_control.jointTarget;
info.JointTargets.MLdataType = 'struct';
info.JointTargets.MaxLen = NaN;
info.JointTargets.MinLen = 0;
data.JointTargets = data.JointTargets([],1);
[data.Condition, info.Condition] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'ob1_arm_control/IKPointsServiceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'pose_targets';
info.MatPath{2} = 'pose_targets.position';
info.MatPath{3} = 'pose_targets.position.x';
info.MatPath{4} = 'pose_targets.position.y';
info.MatPath{5} = 'pose_targets.position.z';
info.MatPath{6} = 'pose_targets.orientation';
info.MatPath{7} = 'pose_targets.orientation.x';
info.MatPath{8} = 'pose_targets.orientation.y';
info.MatPath{9} = 'pose_targets.orientation.z';
info.MatPath{10} = 'pose_targets.orientation.w';
info.MatPath{11} = 'joint_targets';
info.MatPath{12} = 'joint_targets.joint_target';
info.MatPath{13} = 'condition';
