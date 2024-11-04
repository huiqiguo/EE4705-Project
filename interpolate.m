% smallMat and bigMat have the same number of columns, 
% but bigMat has more rows than smallMat;
%
% this is a helper function to interpolate bigMat such that
% interpolatedMat has the same number of rows as smallMat

function interpolatedMat = interpolate(smallMat, bigMat)
   smallDist = linspace(0, 1, size(smallMat, 1));
   bigDist = linspace(0, 1, size(bigMat, 1));

   interpX = interp1(bigDist, bigMat(:, 1), smallDist);
   interpY = interp1(bigDist, bigMat(:, 2), smallDist);
   interpZ = interp1(bigDist, bigMat(:, 3), smallDist);

   interpolatedMat = [interpX', interpY', interpZ'];
end