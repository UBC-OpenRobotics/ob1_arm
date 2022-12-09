function [data, info] = matlabIKServiceRequest
%MatlabIKService gives an empty data for ob1_arm_control/MatlabIKServiceRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ob1_arm_control/MatlabIKServiceRequest';
[data.PoseTarget, info.PoseTarget] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Tolerance, info.Tolerance] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'ob1_arm_control/MatlabIKServiceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'pose_target';
info.MatPath{2} = 'tolerance';
