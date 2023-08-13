function [boat_speed] = hull(boat_reaction, H_par)

boat_speed = sqrt(boat_reaction(1)/H_par);

end