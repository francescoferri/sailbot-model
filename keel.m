function [theta_h_rad] = keel(S_torque, K_length, K_weight, sail_heeling_force_wind, H_heel)

% Keel lift adds to the torque balance to be counteracted
keel_lift_arm = K_length*cos(H_heel); % assumed mid fin

% Total Torque to counteract
M = S_torque + sail_heeling_force_wind*keel_lift_arm;

% get heel angle
theta_h_rad = asin(M/(K_length*K_weight*9.81));

end