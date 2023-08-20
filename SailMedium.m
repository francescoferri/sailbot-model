classdef SailMedium
    
%{ 

Fixed variables that do not change through studies

FluidDensity;
FluidViscosity;
AirfoilMatrix;

Variables that change due to studies

W_speed
W_angle
boat_reaction
boat_speed
boat_heel
S_span
S_avg_ch
S_alpha
S_mass
S_airfoil

%}

    properties
        FluidDensity;
        FluidViscosity;
        AirfoilMatrix;
    end
    
    methods
        function obj = SailMedium(rho_air, mu_air, airfoil_matrix)
            
            %SAILMEDIUM Construct an instance of this class
            
            obj.FluidDensity = rho_air;
            obj.FluidViscosity = mu_air;
            obj.AirfoilMatrix = airfoil_matrix;
        end
        
        function [boat_reaction] = computeSailState(obj, W_speed, W_angle, boat_reaction, boat_speed, boat_heel, S_span, S_avg_ch, S_alpha, S_mass, S_airfoil)
            
            reynolds_number = obj.FluidDensity*S_avg_ch*W_speed/obj.FluidViscosity;
            [cl, cd, cdp, cm] = findAirfoilCoeff(reynolds_number, S_alpha, S_airfoil);
            S_area_heel = S_span*cos(boat_heel)*S_avg_ch; % [m^2] new effective sail area due to heel
            
            % sail force of gravity
            S_gravity = [0 0 -S_mass*9.81];

            [W_app] = getApparentWind(W_speed, W_angle, boat_speed);
            W_app_speed = norm(W_app);
            
            L = 0.5*cl*S_area_heel*W_app_speed^2*obj.FluidDensity; % sail lift
            D = 0.5*cd*S_area_heel*W_app_speed^2*obj.FluidDensity; % sail drag
            
            %% Sail Force: build 2 planes
            % 1. apparent wind direction plane
            % 2. airfoil plane

            % apparent wind plane:
            W_app_dir = W_app/norm(W_app);
            % vertical vector
            vertical = [0 0 1];
            % normal to the apparent wind plane
            n_W_app = cross(W_app_dir, vertical);

            % airfoil plane (same direction as mast)
            n_airfoil = [0 -sin(boat_heel) cos(boat_heel)];
            
            % based on 2 planes, find drag direction
            drag_direction = cross(n_airfoil, n_W_app);
            
            % Using the drag direction and mast direction we find lift
            lift_direction = cross(drag_direction, n_airfoil);

            Lift = lift_direction*L;
            Drag = drag_direction*D;

            S_force = Lift + Drag + S_gravity;

            boat_reaction(1,:) = boat_reaction(1,:) + S_force;

            %% Sail Torques
            % Location of applied aerodynamic force (to switch with COP in the future)
            % is the same as the location of COG of sail
            R = S_span/2;
            Arm = R * n_airfoil;

            S_torque = cross(Arm, S_force);

            boat_reaction(2,:) = boat_reaction(2,:) + S_torque;
        end
    end
end

