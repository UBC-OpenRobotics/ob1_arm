function [data, info] = matlabIKServiceResponse
%MatlabIKService gives an empty data for ob1_arm_control/MatlabIKServiceResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ob1_arm_control/MatlabIKServiceResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.char('string',0);
[data.Error, info.Error] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Joints, info.Joints] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'ob1_arm_control/MatlabIKServiceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'result';
info.MatPath{2} = 'error';
info.MatPath{3} = 'joints';
