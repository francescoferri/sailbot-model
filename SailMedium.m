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
            
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            reynolds_number = obj.FluidDensity*S_avg_ch*W_speed/obj.FluidViscosity;
            
            [cl, cd, cdp, cm] = findAirfoilCoeff(reynolds_number, S_alpha, S_airfoil);
            
            S_area_heel = S_span*cos(boat_heel)*S_avg_ch; % [m^2] new effective sail area due to heel

            [W_app] = getApparentWind(W_speed, W_angle, boat_speed);
            W_app_speed = norm(W_app);
            
            L = 0.5*cl*S_area_heel*W_app_speed^2*obj.FluidDensity; % sail lift
            D = 0.5*cd*S_area_heel*W_app_speed^2*obj.FluidDensity; % sail drag
            
            % Build 2 planes

            % direction of apparent wind
            W_app_dir = W_app/norm(W_app);
            % vertical vector
            vertical = [0 0 1];
            % normal to the apparent wind plane
            n_W_app = cross(W_app_dir, vertical);

            % mast direction (perpendicular to airfoil)
            n_airfoil = [0 -sin(boat_heel) cos(boat_heel)];
            
            % Compute the common direction vector using cross product
            drag_direction = cross(n_airfoil, n_W_app);
            
            % Using the drag direction and mast direction we find lift
            lift_direction = cross(drag_direction, n_airfoil);

            Lift = lift_direction*L;
            Drag = drag_direction*D;

            S_force = Lift + Drag;

            boat_reaction(1,:) = boat_reaction(1,:) + S_force;

            % Finding Torques

            % Location of applied aerodynamic force (to switch with COP in the future)
            % is the same as the location of COG of sail
            R = S_span/2;
            Arm = R*n_airfoil;

            S_torque_aero = cross(Arm, S_force);
            S_torque_mass = cross(Arm, [0 0 -S_mass*9.81]);
            S_torque = S_torque_aero + S_torque_mass;

            boat_reaction(2,:) = boat_reaction(2,:) + S_torque;
        end
    end
end

