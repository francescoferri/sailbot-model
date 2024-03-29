function [boat_stats] = converge(W_speed, W_angle)

initialize

difference = 1;
old_speed = 0;
i = 1;
boat_stats = zeros(1,3);

while (difference > 0.001)

    [boat_reaction] = computeSailState(sailMedium, W_speed, W_angle, boat_reaction, boat_speed, boat_heel, S_span, S_avg_ch, S_alpha, S_mass, S_airfoil);
    [boat_reaction, boat_speed] = computeHullState(hullMedium, boat_reaction, boat_speed);
    [boat_reaction, boat_heel, boat_speed] = computeKeelState(keelMedium, W_speed, W_angle, boat_reaction, boat_speed, boat_heel, K_span, K_avg_ch, K_mass, K_airfoil);
    
    speed = norm(boat_speed);
    difference = abs(speed-old_speed);
    old_speed = speed;

    boat_stats(i,1:3) = [norm(boat_speed)*ms_to_kn rad2deg(boat_heel) difference];
    i = i + 1;

end

figure 1;
plot(boat_stats)

end