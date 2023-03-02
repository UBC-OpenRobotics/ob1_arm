
classdef EEPoseGoals < ros.Message
    %EEPoseGoals MATLAB implementation of relaxed_ik/EEPoseGoals
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'relaxed_ik/EEPoseGoals' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'db34972df908bdbeb860c627c17f6b1e' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'EePoses' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'ee_poses' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.geometry_msgs.Pose' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        EePoses
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'EEPoseGoals', 'Header')
            obj.Header = val;
        end
        function set.EePoses(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.geometry_msgs.Pose.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.geometry_msgs.Pose'};
            validateattributes(val, validClasses, validAttributes, 'EEPoseGoals', 'EePoses')
            obj.EePoses = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.relaxed_ik.EEPoseGoals.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.relaxed_ik.EEPoseGoals(strObj);
        end
    end
end