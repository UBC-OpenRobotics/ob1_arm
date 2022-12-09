%open_system('IKTrajectoryControlExample.slx');

% Import the manipulator as a rigidBodyTree Object
arm = importrobot('main.urdf');
arm.DataFormat = 'column';

% Define end-effector and base link names
eeName = 'ob1_arm_eef_link';
baseName = 'ob1_arm_base_link';

% Define the number of joints in the manipulator
numJoints = 7;

weights = [0.001 0.001 0.001 0.001 0.001 0.001];

randConfig = arm.randomConfiguration;
tform = getTransform(arm,randConfig,eeName,baseName);

% Close all open figures
close all

% Initialize a new figure window
figure;
set(gcf,'Visible','on');

show(arm,randConfig);

% Solve for q0 such that the manipulator begins at the first waypoint
ik = inverseKinematics('RigidBodyTree',arm);
[q0,solInfo] = ik(eeName,tform,weights,arm.homeConfiguration);
disp(solInfo)

% Plot the initial robot position
show(arm, q0);
hold on