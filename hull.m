function [boat_reaction, boat_speed] = hull(boat_reaction, boat_speed, H_par)

% current sailing condition
current_speed_kn = 1.94384*boat_speed;
existing_hull_drag = -H_par*current_speed_kn^2; % pointing backwards (-ve x)

net_hull_drag = existing_hull_drag + boat_reaction(1,1);

boat_speed_kn = sqrt(abs(net_hull_drag)/H_par);
boat_speed = boat_speed_kn/1.94384;

% we assume that the hull can counteract
% 1. reset the net thrust due to hull drag
boat_reaction(1,1) = 0;
% 2. force of gravity (with boiancy)
boat_reaction(1,3) = 0;
% 3. pitching moment with the bow
boat_reaction(2,2) = 0;
% 4. yawing moment with the rudder
boat_reaction(2,3) = 0;

end