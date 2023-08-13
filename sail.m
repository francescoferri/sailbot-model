function [sail_thrust_force,sail_heeling_torque,sail_heeling_force_wind,sail_angle] = sail(rho_air, mu_air, M, W_speed, W_app_angle, S_area, S_avg_ch, S_alpha, S_span, S_mass, boat_heel_rad)

reynolds_number = rho_air*S_avg_ch*W_speed/mu_air;

[cl, cd] = findAirfoilCoeff(reynolds_number, S_alpha, M);

Ls = 0.5*cl*S_area*W_speed^2*rho_air; % sail lift
Ds = 0.5*cd*S_area*W_speed^2*rho_air; % sail drag

sail_angle=W_app_angle-deg2rad(S_alpha);

L_Ls = Ls*cos(boat_heel_rad)*sin(W_app_angle); % Longitudinal Lift
L_Ds = Ds*cos(boat_heel_rad)*cos(W_app_angle); % Longitudinal Drag
sail_thrust_force = L_Ls - L_Ds; % Sail Thrust

T_Ls = Ls*cos(boat_heel_rad)*cos(W_app_angle); % Transverse Lift
T_Ds = Ds*cos(boat_heel_rad)*sin(W_app_angle); % Transverse Drag
sail_heeling_force_wind = T_Ls + T_Ds; % Sail Heeling Force

sail_heeling_force_mass = S_mass*9.81;

sail_heeling_torque = sail_heeling_force_wind*(S_span*cos(boat_heel_rad))/2 + sail_heeling_force_mass*(S_span*sin(boat_heel_rad))/2;

end