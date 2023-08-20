clear;

% Constants
ms_to_kn = 1.94384;
rho_air = 1.225; % air density [kg/m3]
mu_air = 1.81*10^(-5); % dynamic Viscosity [Pa * s]
rho_water = 1020; % water density [kg/m3]
mu_water = 1.308*10^(-3); % dynamic Viscosity [Pa * s]

% Globally changing
W_speed = 0.01; % wind speed [m/s]
W_angle = 0.01; % [rad] wind angle relative to bow, (+ve to the right)

% Extra variables, to insert into individual studies later
W_speed_kn = [2.5; 5; 10; 15; 20]; % wind speed [kn]
W_heading_range = -45:-1:-180; % heading relative to true wind [deg]
W_heading_range(size(W_heading_range,2)) = -179; % modify last value

% coordinate system vectors [x y z]:
boat_force_vector = [0 0 0]; % [N]
boat_torque_vector = [0 0 0]; % [Nm]
boat_reaction = [boat_force_vector; boat_torque_vector];

% boat generated variables
boat_speed = 0.01; % [m/s] initialized to non zero for calculation
boat_heel = -0.01; % [rad] initialized to non zero for calculation

% Import Airfoil Data
naca18 = csvread("xf-naca0018-il-Re-500k-200k-100k-Main.csv",1,0);
% sample call: [cl, cd, cdp, cm] = findAirfoilCoeff(500000, 9, naca18)

% Sail
S_span = 2; % sail height [m]
S_avg_ch = 1; % average chord of sail [m]
S_alpha = 10; % sail angle of attack [deg]
S_mass = 10; % sail mass [kg]
S_airfoil = naca18;
sailMedium = SailMedium(rho_air, mu_air, naca18);

% Hull
% Assumed resistance curve:
H_parameter = 7.5; % hull drag parameter
H_dampener = 0.9;
H_length = 2.35; %LOA [m]
froude_n = 0.7; % defined planing limit
boat_max_speed_kn = froude_n*sqrt(H_length*9.81);% max hull speed [ms]
hullMedium = HullMedium(H_parameter, H_dampener, ms_to_kn);

%Keel
K_span = 2; % Keel length [m]
K_avg_ch = 1; % keel average chord [m]
K_alpha = 0; %[rad] keel angle of attack
K_mass = 60; % Keel weight [kg]
K_airfoil = naca18;
keelMedium = KeelMedium(rho_water, mu_water, naca18);
