function [data, info] = relaxedIKServiceRequest
%RelaxedIKService gives an empty data for relaxed_ik/RelaxedIKServiceRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'relaxed_ik/RelaxedIKServiceRequest';
[data.PoseGoals, info.PoseGoals] = ros.internal.ros.custommessages.relaxed_ik.eEPoseGoals;
info.PoseGoals.MLdataType = 'struct';
info.MessageType = 'relaxed_ik/RelaxedIKServiceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'pose_goals';
info.MatPath{2} = 'pose_goals.header';
info.MatPath{3} = 'pose_goals.header.seq';
info.MatPath{4} = 'pose_goals.header.stamp';
info.MatPath{5} = 'pose_goals.header.stamp.sec';
info.MatPath{6} = 'pose_goals.header.stamp.nsec';
info.MatPath{7} = 'pose_goals.header.frame_id';
info.MatPath{8} = 'pose_goals.ee_poses';
info.MatPath{9} = 'pose_goals.ee_poses.position';
info.MatPath{10} = 'pose_goals.ee_poses.position.x';
info.MatPath{11} = 'pose_goals.ee_poses.position.y';
info.MatPath{12} = 'pose_goals.ee_poses.position.z';
info.MatPath{13} = 'pose_goals.ee_poses.orientation';
info.MatPath{14} = 'pose_goals.ee_poses.orientation.x';
info.MatPath{15} = 'pose_goals.ee_poses.orientation.y';
info.MatPath{16} = 'pose_goals.ee_poses.orientation.z';
info.MatPath{17} = 'pose_goals.ee_poses.orientation.w';
