function [boat_stats, converged_vals] = converge2(true_wind_speed, heading_angle)

ms_to_kn = 1.94384;
boat_speed = 0.5;
i = 1;
force_to_speed_factor = 1/100;
net_force_x = 1;

while (abs(net_force_x) > 0.01 && boat_speed > 0)

    [net_force_x,beta] = optim_approach(boat_speed,true_wind_speed, heading_angle);
    boat_speed = boat_speed + net_force_x*force_to_speed_factor;
    boat_stats(i,1) = boat_speed*ms_to_kn;
    boat_stats(i,2) = beta;
    i = i + 1;

end

converged_vals = [heading_angle,true_wind_speed*ms_to_kn,boat_stats(end,1),boat_stats(end,2)];
%plot(boat_stats())

end