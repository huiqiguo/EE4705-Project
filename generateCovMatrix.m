% helper function to generate a covariance matrix with Gaussian correlation
function sigma = generateCovMatrix(nDiscretize, scale)
    
    % initialise covariance matrix
    sigma = zeros(nDiscretize, nDiscretize); 

    for i = 1:nDiscretize
        for j = 1:nDiscretize
            % Radial basis function
            sigma(i, j) = exp(-((i - j)^2) / (2 * scale^2));
        end
    end
end