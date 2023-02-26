function [data, info] = armCmd
%armCmd gives an empty data for ob1_arm_hw_interface/armCmd
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ob1_arm_hw_interface/armCmd';
[data.Vel, info.Vel] = ros.internal.ros.messages.ros.default_type('single',6);
[data.Angle, info.Angle] = ros.internal.ros.messages.ros.default_type('single',6);
[data.MsgSendCtr, info.MsgSendCtr] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.NumJoints, info.NumJoints] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'ob1_arm_hw_interface/armCmd';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'vel';
info.MatPath{2} = 'angle';
info.MatPath{3} = 'msg_send_ctr';
info.MatPath{4} = 'num_joints';
