%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints' rotation angles at a specific time stamp
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

% % for using MATLAB's builtin getTransform function
% theta_cell = num2cell(theta);
% tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
% [tConfiguration.JointPosition]= theta_cell{:}; % update the joint position using theta

nJoints = length(theta);
T = cell(1,nJoints);
X = zeros(nJoints, 4); 
slist = zeros(6, nJoints); % initialise screw axis matrix

for k=1:nJoints
    %% TODO:
    joint = robot_struct.Bodies{k}.Joint;
    
    % revolute joints
    if strcmp(joint.Type, 'revolute')
        slist(1:3, k) = joint.JointAxis;
        slist(4:6, k) = (robot_struct.Bodies{k}.Joint.JointToParentTransform(1:3, 4));
    % prismatic joints
    elseif strcmp(joint.Type, 'prismatic')
        slist(1:3, k) = joint.JointAxis; 
        slist(4:6, k) = 0; % no rotation for prismatic joints
    % fixed joints
    elseif strcmp(joint.Type, 'fixed')
        slist(:, k) = zeros(6, 1);
    end
    
    % joint angles from joint 1 up to joint k, in a column vector
    theta_k = theta(1:k); 

    % home configuration of base frame
    M = robot_struct.Bodies{1}.Joint.ChildToJointTransform;
    
    T{k} = FKinSpace(M, slist(:, 1:k), theta_k); % compute transformation matrix up to joint k
    
    X(k,1:3) = T{k}(1:3, 4)'; % extract world coordinates of joint k
    X(k, 4) = 1;
end
    
end