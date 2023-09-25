function [boat_stats, converged_vals] = converge2(true_wind_speed, heading_angle, ...
    sail_area,sail_average_chord,keel_area,keel_average_chord)

%Constants
ms_to_kn = 1.94384;
force_to_speed_factor = 1/100;

%Starting Values
boat_speed = 0.5;
i = 1;
net_force_x = 1;

%Loop until force balance is acheived
while (abs(net_force_x) > 0.01 && boat_speed > 0)

    [net_force_x,leeway_angle] = optim_approach(boat_speed, ...
        true_wind_speed, heading_angle, ... %Polar Plot Variables
        sail_area,sail_average_chord,keel_area,keel_average_chord); %Design Parameters
    boat_speed = boat_speed + net_force_x*force_to_speed_factor; %If net force is positive, scale increase boat speed
    %Store Stats
    boat_stats(i,1) = boat_speed*ms_to_kn;
    boat_stats(i,2) = leeway_angle;
    i = i + 1;

end
% Output Results
converged_vals = [true_wind_speed*ms_to_kn,heading_angle,boat_stats(end,1),boat_stats(end,2)];
%plot(boat_stats())

end