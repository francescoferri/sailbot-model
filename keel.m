function [theta_h_rad] = keel(rho_water, mu_water, airfoil_matrix, W_speed, W_app_angle, S_area, S_avg_ch, S_alpha, S_span, S_mass, boat_heel_rad)

% Keel lift adds to the torque balance to be counteracted by the mass

reynolds_number = rho_water*K_avg_ch*boat_speed_ms/mu_water;

%{
 
what is the lift needed to keep the boat sailing straight? (and not drift
sideways like a baloon on the water)

1. get the global Y force to counteract
2. begin increasing the keels angle of attack
3. go get coeficients based on current alpha
4. find keel_lift, normalize onto keel_y_lift using current heel angle
5. repeat 2-4 untill magnitude of keel_y_lift is equal to the global Y force

%}

[cl, cd] = findAirfoilCoeff(reynolds_number, K_alpha, airfoil_matrix);

Ls = 0.5*cl*S_area*W_speed^2*rho_air; % sail lift
Ds = 0.5*cd*S_area*W_speed^2*rho_air; % sail drag

sail_angle=W_app_angle-deg2rad(S_alpha);

L_Ls = Ls*cos(H_heel)*sin(W_app_angle); % Longitudinal Lift
L_Ds = Ds*cos(H_heel)*cos(W_app_angle); % Longitudinal Drag
sail_thrust_force = L_Ls - L_Ds; % Sail Thrust

T_Ls = Ls*cos(H_heel)*cos(W_app_angle); % Transverse Lift
T_Ds = Ds*cos(H_heel)*sin(W_app_angle); % Transverse Drag
sail_heeling_force_wind = T_Ls + T_Ds; % Sail Heeling Force

sail_heeling_force_mass = S_mass*9.81;

sail_heeling_torque = sail_heeling_force_wind*(S_span*cos(H_heel))/2 + sail_heeling_force_mass*(S_span*sin(H_heel))/2;

keel_lift_arm = K_length*cos(H_heel); % assumed mid fin

% Total Torque to counteract
M = S_torque + sail_heeling_force_wind*keel_lift_arm;

% get heel angle
theta_h_rad = asin(M/(K_length*K_weight*9.81));

end