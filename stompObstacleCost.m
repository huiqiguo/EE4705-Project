function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)
% this function computes the obstacle cost at a given time stamp
% size(cost) = [nJoints, 1]

safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz indices in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);

%% TODO: complete the following code according to Eq (13) in the STOMP conference paper.
try
    % dxb is a col vector, each value in dxb represents the sEDT for each sphere
    % nrows of dxb = nspheres
    dxb = voxel_world_sEDT(sub2ind(size(voxel_world_sEDT), idx(:, 1), idx(:, 2), idx(:, 3))); 

    cost_array = zeros(length(sphere_centers), 1); % initialise empty col vector to store costs
    
    for sphere = 1:length(sphere_centers) % iterate through all the spheres
        sphereCost = max([safety_margin + radius - dxb(sphere), 0])*vel(sphere); % obstacle cost for this sphere
        cost_array(sphere) = sphereCost; % append to the cost array
    end
    
    % sample points from the cost array such that number of points = nJoints
    cost = cost_array(round(linspace(1, length(cost_array), numJoints)));
    
catch  % for debugging
    idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
end