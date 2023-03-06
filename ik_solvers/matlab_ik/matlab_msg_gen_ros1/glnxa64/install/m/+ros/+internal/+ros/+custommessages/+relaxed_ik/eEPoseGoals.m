function [data, info] = eEPoseGoals
%EEPoseGoals gives an empty data for relaxed_ik/EEPoseGoals
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'relaxed_ik/EEPoseGoals';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.EePoses, info.EePoses] = ros.internal.ros.messages.geometry_msgs.pose;
info.EePoses.MLdataType = 'struct';
info.EePoses.MaxLen = NaN;
info.EePoses.MinLen = 0;
data.EePoses = data.EePoses([],1);
info.MessageType = 'relaxed_ik/EEPoseGoals';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'ee_poses';
info.MatPath{8} = 'ee_poses.position';
info.MatPath{9} = 'ee_poses.position.x';
info.MatPath{10} = 'ee_poses.position.y';
info.MatPath{11} = 'ee_poses.position.z';
info.MatPath{12} = 'ee_poses.orientation';
info.MatPath{13} = 'ee_poses.orientation.x';
info.MatPath{14} = 'ee_poses.orientation.y';
info.MatPath{15} = 'ee_poses.orientation.z';
info.MatPath{16} = 'ee_poses.orientation.w';
