function [net_force_x, beta] = optim_approach(boat_speed, true_wind_speed,heading_angle)
%Permanent Constants
air_density = 1.225; % [kg/m3]
air_viscosity = 1.81*10^(-5); % [Pa * s]
water_density = 1020; % [kg/m3]
water_viscosity =  1.308*10^(-3); % [Pa * s]

% Import Airfoil Data
naca18 = csvread("xf-naca0018-il-Re-500k-200k-100k-Main.csv",1,0);

%Design Paramters
sail_area = 3; % m^2
sail_average_chord = 1; % m
keel_area = 0.5; % m^2
keel_average_chord = keel_area/2; % m

%Temporary Constants
leeway_angle = 0; % Degrees
sail_angle_of_attack = 10; % Degrees

%Input Variables
current_speed = 0; % [m/s]
current_angle = 0; % Degrees (from hull coordinates)

%Output Variable
%boat_speed; % [m/s]

%Sail Forces In Sail Coordinates

apparent_wind_speed = sqrt(boat_speed^2 + true_wind_speed^2);
reynolds_number_air = apparent_wind_speed*air_density*sail_average_chord/air_viscosity;
[cl_air, cd_air, cdp_air, cm_air, at_stall_air] = findAirfoilCoeff(reynolds_number_air,sail_angle_of_attack,naca18);

sail_lift_force = 0.5*air_density*apparent_wind_speed^2*sail_area*abs(cl_air);
sail_drag_force = 0.5*air_density*apparent_wind_speed^2*sail_area*abs(cd_air);

%Sail Coordinate Transformation
apparent_wind_angle = atan2d(true_wind_speed*sind(heading_angle),(boat_speed+cosd(heading_angle)*true_wind_speed));
sail_x_force = sail_lift_force*sind(apparent_wind_angle) + -1*sail_drag_force*cosd(apparent_wind_angle);
sail_y_force = -1*sail_lift_force*cosd(apparent_wind_angle) + -1*sail_drag_force*sind(apparent_wind_angle);

%Hull Forces in Hull Coordinates

hull_drag = hull_resistance_curve(boat_speed);



%Keel Forces in Water Cordinates

apparent_water_speed = sqrt(boat_speed^2 + current_speed^2);

reynolds_number_water = apparent_water_speed*water_density*keel_average_chord/water_viscosity;

%Find Keel CL From Sail and Hull Forces then find leeway method

cl_water = 2*sail_y_force/(water_density*apparent_water_speed^2*keel_area);
beta = findLeeway(reynolds_number_water,cl_water,naca18);


%Controlled leeway method

[cl_water_check, cd_water, cdp_water, cm_water, at_stall_water] = findAirfoilCoeff(reynolds_number_water,beta,naca18);


keel_drag_force = 0.5*water_density*apparent_water_speed^2*keel_area*abs(cd_water);
keel_lift_force = 0.5*water_density*apparent_water_speed^2*keel_area*abs(cl_water);

%Keel Coordinate Transformation

keel_x_force = -1*keel_lift_force*sind(beta) + -1*keel_drag_force*cosd(beta);
keel_y_force = keel_lift_force*cosd(beta) + -1*keel_drag_force*sind(beta);

%}

%Force Balance / Solve

net_force_x = sail_x_force - hull_drag + keel_x_force;
net_force_y = sail_y_force + keel_y_force;

end

%Constant Functions
function [hull_drag] =  hull_resistance_curve(boat_speed) 
    hull_drag = 7.5*boat_speed^2;
end


 



