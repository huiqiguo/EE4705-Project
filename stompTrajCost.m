% Given a trajectory, calculate its cost
function [Stheta, Qtheta] = stompTrajCost(robot_struct, theta,  R, voxel_world)
% Compute the local trajectory cost at each discretization theta point (Stheta), 
% as well as the overall trajectory cost (the Qtheta)

% Costi = stompCompute_Cost(robot, theta, Env);
% Compute the cost of all discretized points on one trajectory
[nJoints, nDiscretize] = size(theta); % nDiscretize = number of discretized time points for each joint
% Obstacle cost 
qo_cost = cell(1, nDiscretize); % obstacle cost of the whole robot manipulator, across various time points
% Constraint costs
qc_cost = zeros(1, nDiscretize); % constraint cost of the whole robot manipulator, across various time points

% first time stamp
% Get the coordinates of joints in world frame 
[X, ~] = updateJointsWorldPosition(robot_struct, theta(:, 1));
% Construct the spheres around the robot manipulator for collision avoidance
[sphere_centers,radi] = stompRobotSphere(X);
% Initial velocity at the sphere centers around the manipulator is 0
vel = zeros(length(sphere_centers), 1);
qo_cost{1} = stompObstacleCost(sphere_centers, radi, voxel_world, vel, nJoints);

% iterate through all time stamps
for i = 2 : nDiscretize
    sphere_centers_prev = sphere_centers;
    % Calculate the kinematics of the manipulator, given the
    % configuration theta values at different time (i=2:nDiscretize)
    [X, ~] = updateJointsWorldPosition(robot_struct, theta(:, i));
    [sphere_centers, radi] = stompRobotSphere(X);

    %% TO IMPROVE
    % number of sphere centers might be different across iterations,
    % hence this section interpolates the larger array to match the number of points in the smaller array;
    % works but very slow
    if size(sphere_centers, 1) < size(sphere_centers_prev, 1)
        sphere_centers_prev = interpolate(sphere_centers, sphere_centers_prev);
    elseif size(sphere_centers, 1) > size(sphere_centers_prev, 1)
        sphere_centers = interpolate(sphere_centers_prev, sphere_centers);
    end

    %%
    % xb: 3D workspace position of sphere b at the current time
    % Approximate the speed (xb_dot) using the finite difference of the current and
    % the previous position
    vel = vecnorm(sphere_centers_prev - sphere_centers,2,2);
    qo_cost{i} = stompObstacleCost(sphere_centers, radi, voxel_world, vel, nJoints);
    
    %% TODO: Define your qc_cost to add constraint on the end-effector
    % scenario 1: no constraints
    % qc_cost(i) = 0;

    % scenario 2: align y-axis of end-effector to the global z-axis
    endEffectorCoords = X(end, :); % extract the last row from X to obtain the end effector coords
    prevJointCoords = X(end-1, :); % extract the second last row from X to obtain the coords of the second last joint
    endEffectorDir = (endEffectorCoords - prevJointCoords) / norm(endEffectorCoords - prevJointCoords); % approximate the direction of the end effector
    difference = [0, 0, 1, 1] - endEffectorDir; % difference vector betw global z-axis and end-effector direction
    qc_cost(i) = dot(difference, difference); % l2 norm as a measure of constraint cost
end

%% Local trajectory cost: you need to specify the relative weights between different costs
qo_cost = cell2mat(qo_cost);
Stheta = 1000*qo_cost + qc_cost; % size(Stheta) = [nJoints, nDiscretize]

% sum over time and add the smoothness cost
theta = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta * R * theta', "all");

end
