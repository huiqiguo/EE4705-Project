%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints' rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

% % for using MATLAB's builtin getTransform function
% theta_cell = num2cell(theta);
% tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
% [tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta

% get the number of joints
nJoints = length(theta);
T = cell(1,nJoints);
X = zeros(nJoints, 3); 

for k=1:nJoints
    %% TODO:
    slist_k = robot_struct.Slist(:, 1:k); % get the list of screw axes from joint 1 up to joint k
    theta_k = theta(1:k); % get the joint angles from joint 1 up to joint k
    M_k = robot_struct.M{k}; % home configuration for joint k
    
    T{k} = FKinSpace(M_k, slist_k, theta_k); % compute transformation matrix up to joint k
    
    X(k,:) = T{k}(1:3, 4)'; % extract world coordinates of joint k
end
    
end