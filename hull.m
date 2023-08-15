function [boat_reaction, boat_speed] = hull(boat_reaction, H_par)

boat_speed_kn = sqrt(boat_reaction(1,1)/H_par);
boat_speed = boat_speed_kn/1.94384;

% we assume that the hull can counteract
% 1. thrust provided
boat_reaction(1,1) = 0;
% 2. force of gravity (with boiancy)
boat_reaction(1,3) = 0;
% 3. pitching moment with the bow
boat_reaction(2,2) = 0;
% 4. yawing moment with the rudder
boat_reaction(2,3) = 0;

end