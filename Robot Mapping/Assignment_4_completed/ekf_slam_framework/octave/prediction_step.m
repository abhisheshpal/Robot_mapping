function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
mu(1) = mu(1) + u.t*cos(mu(3) + u.r1);  %here we changing current mean state from previous beleive
mu(2) = mu(2) + u.t*sin(mu(3) + u.r1); % changing second element in matrix 3x3
mu(3) = mu(3) + (u.r2 + u.r1);       % changing second element in matrix 3x3

% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gxt = eye(3);   % making Identity matrix of 'Gxt' % based on updated motion-motion model the jacobian matrix is calculated by partially diffrentiating the motion model  
Gxt(1,3) = -u.t*sin(mu(3) + u.r1);   % Changing row1 and 3rd column of Idenetity Gxt matrix
Gxt(2,3) = u.t*cos(mu(3) + u.r1);    % Changing row2 and 3rd column of Idenetity Gxt matrix

% TODO: Construct the full Jacobian G
Jacob_robSigma = Gxt; % creating 3x3 jacobian matrix of robot motion model
Jacob_robMapSigma = zeros(3,18); % creating 3x18 zeros as input jacobian matrix for robotand map landmarks relations
Jacob_mapSigma = eye(18); % creating 18x18 Identity matrix for landmarks only 
gsigma = [[Jacob_robSigma Jacob_robMapSigma];[Jacob_robMapSigma' Jacob_mapSigma]];

% Motion noise
motionNoise = 0.1;     % cosidered motion noise as 0.1
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion
sigma = gsigma*sigma*gsigma' + R;  % here adding all the jacobians and motion noise
end
