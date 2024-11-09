%Parameters
% T = 5; 
nDiscretize = 20; % number of discretized waypoint
nPaths = 20; % number of sample paths
convergenceThreshold = 0.1; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of q0 and qT
q0 = currentRobotJConfig;
qT = finalRobotJConfig;
numJoints = length(q0);
theta=zeros(numJoints, nDiscretize);
for k=1:length(q0)
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

% store sampled paths
theta_samples = cell(1,nPaths);

%% for calculating the acceleration of theta
% Precompute
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize); % identity matrix with -2 as the diagonal
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k; % sets the diagonal above the -2 diagonal to be all 1s
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k; % sets the diagonal below the -2 diagonal to be all 1s
A = A(:, 2:end-1); % remove the first and last column of A, obtain the finite difference matrix
R = A' * A; % multiply A by its transpose, R is symmetric
Rinv = inv(R); % also symmetric
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); % normalized by each column, no longer symmetric
Rinv = 1.5*Rinv/sum(sum(Rinv)); % normalized R inverse, so that the sample is still within the voxel world

%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;

iter=0; 
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    % overall cost: Qtheta
    QthetaOld = Qtheta;
    % use tic and toc for printing out the running time
    tic
    %% TODO: Complete the following code. The needed functions are already given or partially given in the folder.
    %% TODO: Sample noisy trajectories
    % generate covariance matrix of size [nDiscretize, nDiscretize]
    scale = 10; % tunable parameter for smoothness
    sigma = generateCovMatrix(nDiscretize, scale);
    [sampledTrajectories, noise] = stompSamples(nPaths, sigma, theta);

    %% TODO: Calculate Local trajectory cost for each sampled trajectory
    
    % size(localCost) = [nJoints, 1]
    % size(each cell) = [nPaths, nDiscretize]
    localCost = cell(numJoints, 1); 
    for i = 1:numJoints
        localCost{i} = cell(nPaths, nDiscretize);
    end

    % loop through each sampled path and calculate its cost
    for i = 1:nPaths
        theta_i = sampledTrajectories{i};

        % size(scost_i) = [nJoints, nDiscretize]
        [scost_i, qcost_i] = stompTrajCost(robot_struct, theta_i, R, voxel_world);

        % store costs according to required dimensions
        for j = 1:numJoints
            for k = 1:nDiscretize
                localCost{j}{i, k} = scost_i(j, k);
            end
        end
    end
    
    %% TODO: Given the local traj cost, update local trajectory probability

    probabilities = cell(numJoints, 1);

    for i = 1:numJoints
        probabilities{i} = stompUpdateProb(cell2mat(localCost{i}));
    end
    
    %% TODO: Compute delta theta (aka gradient estimator, the improvement of the delta)
    dTheta = stompDTheta(probabilities, noise);

    %% TODO: Compute the new trajectory
    theta = stompUpdateTheta(theta, dTheta, Rinv);
    
    %% TODO: Compute the total cost of the new trajectory
    [~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);

    %%
 
    toc

    Q_time = [Q_time Qtheta];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Set the stopping iteration criteria:
    if iter > 50 
        disp('Maximum iteration (50) has reached.')
        break
    end

    dtheta_smoothed = movmean(dTheta, 3, 2); % Smooth along the second dimension (waypoints)

    if sum(dtheta_smoothed,'all') == 0
        disp('Estimated gradient is 0 and Theta is not updated: there could be no obstacle at all')
        break
    end
end

disp('STOMP Finished.');

%% check collision
inCollision = false(nDiscretize, 1); % initialize the collision status vector
worldCollisionPairIdx = cell(nDiscretize,1); % Initialization: Provide the bodies that are in collision

for i = 1:nDiscretize

    [inCollision(i),sepDist] = checkCollision(robot,theta(:,i),world,"IgnoreSelfCollision","on","Exhaustive","on");

    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx];
    worldCollisionPairIdx{i} = worldCollidingPairs;

end
isTrajectoryInCollision = any(inCollision)

%% Plot training dynamics
enableVideoTraining = 0;

v = VideoWriter('KinovaGen3_Training.avi');
v.FrameRate = 15;
open(v);

% htext = text(-0.2,0.6,0.7,'Iteration = 0','HorizontalAlignment','left','FontSize',14);

if enableVideoTraining == 1
    theta_animation_tmp = theta_animation(~cellfun('isempty',theta_animation));
    nTraining = length(theta_animation_tmp);
    for k=0:5:nTraining
        
        UpdatedText = ['Iteration = ',num2str(k)];
        set(htext,'String',UpdatedText)
        theta_tmp = theta_animation_tmp{k+1};

        for t=1:size(theta_tmp,2)
            show(robot, theta_tmp(:,t),'PreservePlot', false, 'Frames', 'on');
            %             drawnow;
            frame = getframe(gcf);
            writeVideo(v,frame);
%             pause(1/15);
            %     pause;
        end
        pause(1/15);
    end
end
close(v);

%% Plot path
enableVideo = 1;
if enableVideo == 1
    v = VideoWriter('KinovaGen3.avi');
    v.FrameRate =2;
    open(v);

    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        frame = getframe(gcf);
        writeVideo(v,frame);
        pause(5/20);
        %     pause;
    end
    close(v);
end

%%
displayAnimation = 1;
if displayAnimation
    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        pause(5/20);
        %     pause;
    end
end

%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')
