% TASK 4: Generate obstacles for the KukaIiwa robot manipulator

%% Construct voxel obstacle representation for STOMP planner
% Define the workspace (the voxel world limits). Depending on the robot's reach range.
Env_size = [-1, -1, -1; 2, 2, 2];  % [xmin, ymin, zmin] for 1st row, xyz-lengths for 2nd row
voxel_size = [0.02, 0.02, 0.02];  % unit: m
% Binary map: all free space initially
binary_world = zeros(Env_size(2, 1) / voxel_size(1), Env_size(2, 2) / voxel_size(2), Env_size(2, 3) / voxel_size(3));
% binary_world_offset = Env_size(1, :)./ voxel_size;
%% XYZ metric representation (in meter) for each voxel 
% !!!!Watch out for the useage of meshgrid:
% [X,Y,Z] = meshgrid(x,y,z) returns 3-D grid coordinates defined by the 
% vectors x, y, and z. The grid represented by X, Y, and Z has size 
% length(y)-by-length(x)-by-length(z).
% The 3D coordinate is of the center of the voxels
[Xw, Yw, Zw] = meshgrid(Env_size(1, 1) + 0.5 * voxel_size(1) : voxel_size(1) : Env_size(1, 1) + Env_size(2, 1) - 0.5 * voxel_size(1), ...
       Env_size(1, 2) + 0.5 * voxel_size(2) : voxel_size(2) : Env_size(1, 2) + Env_size(2, 2) - 0.5 * voxel_size(2), ...
    Env_size(1, 3) + 0.5 * voxel_size(3) : voxel_size(3) : Env_size(1, 3) + Env_size(2, 3) - 0.5 * voxel_size(3));

%% Static obstacles
lboxes = [0.16; 0.2]; % length of cubes
boxCenters = [0.45, 0.2, 0.3; 0.23, -0.2, 0.2]; % world coordinates of box centers
world = {};

for i = 1:length(lboxes)
    obs = collisionBox(lboxes(i), lboxes(i), lboxes(i));
    obs.Pose = trvec2tform(boxCenters(i, :));
    world{i} = obs;
end

%% Visulaization the obstacle
% for i=1: length(world)
%     show(world{i})
% end

%% voxelize the box obstacles
for i = 1:length(lboxes)
    lbox = lboxes(i);
    boxCenter = boxCenters(i, :);
    cubeMetric = [boxCenter-lbox/2; % [xmin, ymin, zmin] for 1st row, xyz-lengths for 2nd row
                    lbox,lbox,lbox];
    cubeVoxel = [ceil((cubeMetric(1, :)-Env_size(1,:))./voxel_size); ... % range (lower and upper limits) of the cube voxel
        ceil((cubeMetric(1, :) + cubeMetric(2, :)-Env_size(1,:))./voxel_size)];
    % Update the cube occupancy in the voxel world
    [xc, yc, zc] = meshgrid(cubeVoxel(1, 1):cubeVoxel(2, 1), cubeVoxel(1, 2):cubeVoxel(2, 2), cubeVoxel(1, 3):cubeVoxel(2, 3));
    binary_world(sub2ind([Env_size(2, 1) / voxel_size(1), Env_size(2, 2) / voxel_size(2), Env_size(2, 3) / voxel_size(3)], xc, yc, zc)) = 1;
end

% % plot the occupied voxel with a marker *
% plot3(xc(:), yc(:), zc(:), '*');
% % Or you can use the volumeViewer() from the Image Processing Toolbox to 
% % display the voxel_world in 3D. 
% volumeViewer(voxel_world);

%% construct signed Euclidean Distance for the voxel world
% Only approximation if the voxel is not a cube
voxel_world_sEDT = prod(voxel_size) ^ (1/3) * sEDT_3d(binary_world);
voxel_world.voxel_size = voxel_size;
voxel_world.voxel = binary_world;
% voxel_world.offset =  binary_world_offset;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size = Env_size; % in metric 
voxel_world.sEDT =  voxel_world_sEDT;
