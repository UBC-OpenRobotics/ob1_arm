function [data, info] = iKPointsServiceRequest
%IKPointsService gives an empty data for ob1_arm_control/IKPointsServiceRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'ob1_arm_control/IKPointsServiceRequest';
[data.Request, info.Request] = ros.internal.ros.messages.ros.char('string',0);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose;
info.Pose.MLdataType = 'struct';
[data.NumPts, info.NumPts] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Tolerance, info.Tolerance] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'ob1_arm_control/IKPointsServiceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'request';
info.MatPath{2} = 'pose';
info.MatPath{3} = 'pose.position';
info.MatPath{4} = 'pose.position.x';
info.MatPath{5} = 'pose.position.y';
info.MatPath{6} = 'pose.position.z';
info.MatPath{7} = 'pose.orientation';
info.MatPath{8} = 'pose.orientation.x';
info.MatPath{9} = 'pose.orientation.y';
info.MatPath{10} = 'pose.orientation.z';
info.MatPath{11} = 'pose.orientation.w';
info.MatPath{12} = 'num_pts';
info.MatPath{13} = 'tolerance';
info.MatPath{14} = 'distance';
