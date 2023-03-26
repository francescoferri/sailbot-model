function [H_speed] = hull(H_drag, H_par)

H_speed = sqrt(H_drag/H_par);

end