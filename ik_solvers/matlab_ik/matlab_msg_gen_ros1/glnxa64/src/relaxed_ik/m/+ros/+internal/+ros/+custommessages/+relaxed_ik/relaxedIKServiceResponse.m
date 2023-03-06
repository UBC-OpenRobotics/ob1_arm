function [data, info] = relaxedIKServiceResponse
%RelaxedIKService gives an empty data for relaxed_ik/RelaxedIKServiceResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'relaxed_ik/RelaxedIKServiceResponse';
[data.JointAngles, info.JointAngles] = ros.internal.ros.custommessages.relaxed_ik.jointAngles;
info.JointAngles.MLdataType = 'struct';
info.MessageType = 'relaxed_ik/RelaxedIKServiceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'joint_angles';
info.MatPath{2} = 'joint_angles.header';
info.MatPath{3} = 'joint_angles.header.seq';
info.MatPath{4} = 'joint_angles.header.stamp';
info.MatPath{5} = 'joint_angles.header.stamp.sec';
info.MatPath{6} = 'joint_angles.header.stamp.nsec';
info.MatPath{7} = 'joint_angles.header.frame_id';
info.MatPath{8} = 'joint_angles.angles';
info.MatPath{9} = 'joint_angles.angles.data';
