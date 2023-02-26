function [data, info] = armState
%armState gives an empty data for ob1_arm_hw_interface/armState
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ob1_arm_hw_interface/armState';
[data.Angle, info.Angle] = ros.internal.ros.messages.ros.default_type('single',6);
[data.Vel, info.Vel] = ros.internal.ros.messages.ros.default_type('single',6);
[data.MsgRcvCtr, info.MsgRcvCtr] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.BufferHealth, info.BufferHealth] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'ob1_arm_hw_interface/armState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'angle';
info.MatPath{2} = 'vel';
info.MatPath{3} = 'msg_rcv_ctr';
info.MatPath{4} = 'buffer_health';
