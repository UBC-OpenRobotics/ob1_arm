
classdef JointAngles < ros.Message
    %JointAngles MATLAB implementation of relaxed_ik/JointAngles
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'relaxed_ik/JointAngles' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '3d19a5bafff81501c7a05b5f8ee0a31e' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Angles' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'angles' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.std_msgs.Float32' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Angles
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'JointAngles', 'Header')
            obj.Header = val;
        end
        function set.Angles(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.std_msgs.Float32.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.std_msgs.Float32'};
            validateattributes(val, validClasses, validAttributes, 'JointAngles', 'Angles')
            obj.Angles = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.relaxed_ik.JointAngles.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.relaxed_ik.JointAngles(strObj);
        end
    end
end
