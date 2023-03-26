function [apparent_wind_angle] = getApparentWind(W_speed, H_speed, H_heading)

% finding apparent wind angle
% angles relative to bow +ve on stbd side
% x-axis pointing foreward, y-axis pointing port
head_wind = [-H_speed 0];
true_wind = [-W_speed*cos(H_heading) W_speed*sin(H_heading)];
apparent_wind = true_wind + head_wind;
% getting angle with atan
if (apparent_wind(1)<0)
    apparent_wind_angle = atan(abs(apparent_wind(2)/apparent_wind(1)));
elseif (apparent_wind(1)>0)
    apparent_wind_angle = pi - atan(abs(apparent_wind(2)/apparent_wind(1)));
else
    apparent_wind_angle = pi/2;
end

end