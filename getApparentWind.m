function [apparent_wind] = getApparentWind(W_speed, W_angle, boat_speed)

% finding apparent wind angle
true_wind = [-W_speed*cos(W_angle) -W_speed*sin(W_angle)];
head_wind = [-boat_speed 0];
apparent_wind = true_wind + head_wind;

end