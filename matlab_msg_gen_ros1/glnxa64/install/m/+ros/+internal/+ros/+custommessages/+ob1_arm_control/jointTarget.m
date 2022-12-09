function [data, info] = jointTarget
%JointTarget gives an empty data for ob1_arm_control/JointTarget
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ob1_arm_control/JointTarget';
[data.JointTarget_, info.JointTarget_] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'ob1_arm_control/JointTarget';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'joint_target';
