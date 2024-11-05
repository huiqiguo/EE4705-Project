% dtheta: estimated gradient, summed across all trajectories
% size(dtheta) = [nJoints, nDiscretize]
%
% trajProb: nJoints by 1 cell, each cell is nSamples by nDiscretize matrix
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix

function dtheta = stompDTheta(trajProb, em)

nJoints = length(trajProb);
nDiscretize = size(trajProb{1}, 1);

% variable declaration
dtheta = zeros(nJoints, nDiscretize);

%% TODO: iterate over all joints to compute dtheta: (complete your code according to the STOMP algorithm)
for i = 1:nJoints
    prob_i = trajProb{i};
    noise_i = em{i};
    dtheta(i, :) = sum(prob_i .* noise_i, 1);
end