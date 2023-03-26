function [theta_h_rad] = keel(Ms, K_length, K_weight)

% get heel angle
theta_h_rad = asin(Ms/(K_length*K_weight*9.81));

end