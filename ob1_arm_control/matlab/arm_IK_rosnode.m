rosinit('http://localhost:11311');

srv = rossvcserver("/matlab_ik","ob1_arm_control/MatlabIKService",@serviceCallback);

function response = serviceCallback(src,reqMsg,defaultRespMsg)
    % Import the manipulator as a rigidBodyTree Object
    arm = importrobot('main.urdf');
    arm.DataFormat = 'column';
    
    % Define end-effector and base link names
    eeName = 'ob1_arm_eef_link';
    baseName = 'ob1_arm_base_link';
    
    % Define the number of joints in the manipulator
    numJoints = 7;
    
    % Solve for q0 such that the manipulator begins at the first waypoint
    ik = inverseKinematics('RigidBodyTree',arm);

    response = rosmessage('ob1_arm_control/MatlabIKServiceResponse');

    rpy = reqMsg.PoseTarget(4:6);
    xyz = reqMsg.PoseTarget(1:3);

    %create homogenous 4x4 tform matrix of pose
    tPose = eul2tform(rpy')
    tPose(1:3,end) = xyz

    [q0,solInfo] = ik(eeName,tPose,reqMsg.Tolerance,arm.homeConfiguration)

    response.Result = solInfo.Status
    response.Error = solInfo.PoseErrorNorm
    response.Joints = q0
end