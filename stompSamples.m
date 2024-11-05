% Input: 
%   nSamplePaths: the number of sample paths 
%   sigma： sample covariance matrix, size = [nDiscretize, nDiscretize]
%   theta: mean trajectory from last iteration, size = [nJoints, nDiscretize]
% Output:
%   theta_paths: sampled trajectories
%   size(theta_paths) = [1, nSamplePaths], size(each cell) = [nJoints, nDiscretize]
%   em: sampled Gaussian noise for each joint
%   size(em) = [1, nJoints], size(each cell) = [nSamplePaths, nDiscretize]

function [theta_paths, em] = stompSamples(nSamplePaths, sigma, theta)
% Sample theta (joints angles) trajectory 

[nJoints, nDiscretize] = size(theta);

em = cell(1,nJoints);
ek = cell(1,nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu = zeros(1,length(sigma)); % noise has zero mean

%% TODO: complete code of independent sampling for each joint
% Each joint is sampled independently.
% The starting q0 and final qT are fixed, so set the sample to 0.
for m = 1 : nJoints
    em{m} = mvnrnd(mu, sigma, nSamplePaths); % sample from multivariable Gaussian distribution
    em{m}(:, 1) = 0; % set starting noise as zero
    em{m}(:, end) = 0; % set final noise as zero
end

%% Regroup it by samples
emk = [em{:}]; % size = [nSamplePaths, nJoints*nDiscretize]
for k = 1:nSamplePaths
    % emk(k, :) extracts all noise values for sample path k, size = [1, nJoints*nDiscretize]
    % reshape the row vector into a matrix of size [nDiscretize, nJoints]
    % transpose to make the size [nJoints, nDiscretize] so that it matches theta
    ek{k} = reshape(emk(k,:), nDiscretize, nJoints)';
    theta_paths{k} = theta + ek{k};
end