function [data, info] = jointAngles
%JointAngles gives an empty data for relaxed_ik/JointAngles
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'relaxed_ik/JointAngles';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Angles, info.Angles] = ros.internal.ros.messages.std_msgs.float32;
info.Angles.MLdataType = 'struct';
info.Angles.MaxLen = NaN;
info.Angles.MinLen = 0;
data.Angles = data.Angles([],1);
info.MessageType = 'relaxed_ik/JointAngles';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'angles';
info.MatPath{8} = 'angles.data';
