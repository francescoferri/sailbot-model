classdef KeelMedium
    
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
K_span
K_avg_ch
K_mass
K_airfoil

%}

    properties
        FluidDensity;
        FluidViscosity;
        AirfoilMatrix;
    end
    
    methods
        function obj = KeelMedium(rho_water, mu_water, airfoil_matrix)
            
            %SAILMEDIUM Construct an instance of this class
            
            obj.FluidDensity = rho_water;
            obj.FluidViscosity = mu_water;
            obj.AirfoilMatrix = airfoil_matrix;
        end
        
        function [boat_reaction, boat_heel] = computeKeelState(obj, W_speed, W_angle, boat_reaction, boat_speed, boat_heel, K_span, K_avg_ch, K_mass, K_airfoil)
            
            K_torque_debt = boat_reaction(2,1)*2; % assumed a perfect
            % force couple moment about the x-axis (sail force = keel force in y direction)

            boat_heel_new = asin(K_torque_debt/(K_span*K_mass*9.81));
            heel_delta = boat_heel_new - boat_heel;
            boat_heel = boat_heel + heel_delta;
            boat_reaction(2,1) = 0;
            
            %{
 get some initial data
            reynolds_number = obj.FluidDensity*K_avg_ch*boat_speed/obj.FluidViscosity;
            K_area = K_span * K_avg_ch; % [m2] assumed to be square
            
            boat_drift_force = boat_reaction(1,2); % y-axis force to counteract with the keel
            
            % keel force of gravity
            K_gravity = [0 0 -K_mass*9.81];

            % airfoil plane (keel's direction)
            n_airfoil = [0 sin(boat_heel) -cos(boat_heel)];

            K_force_hydro = [0 0 0];
            K_alpha = 0; % [deg]
            
            %% what angle of attack is required for the boat not to drift?
            while (abs(K_force_hydro(2)) < abs(boat_drift_force))

                K_alpha = K_alpha + 0.01; % [deg] increase angle of attack
    
                % find matching coeficient
                [cl, cd, cdp, cm] = findAirfoilCoeff(reynolds_number, K_alpha, K_airfoil);
                
                L = 0.5*cl*K_area*boat_speed^2*obj.FluidDensity; % keel lift
                D = 0.5*cd*K_area*boat_speed^2*obj.FluidDensity; % keel drag
                
                %% Keel Force: build 2 planes
                
                % water plane
                % water direction
                water = [-boat_speed*cos(deg2rad(K_alpha)) -boat_speed*sin(deg2rad(K_alpha)) 0];
                water_dir = water/norm(water);
                % vertical
                vertical = [0 0 1];
                % normal to water plane
                n_water = cross(water_dir, vertical);
                
                % using the two planes compute direction of drag
                drag_direction = cross(n_water, n_airfoil);
                  
                % Using the drag direction and keel direction we find lift
                lift_direction = cross(drag_direction, n_airfoil);
    
                Lift = lift_direction*L;
                Drag = drag_direction*D;
    
                K_force_hydro = Lift + Drag;

            end

            %% Record changes into force balance
            K_force = K_force_hydro + K_gravity;
            boat_reaction(1,:) = boat_reaction(1,:) + K_force;

            %% Keel Torques
            % Location of applied hydro force (switch with COP in the future)
            Arm_hydro = K_span/2 * n_airfoil;
            K_torque_hydro = cross(Arm_hydro, K_force_hydro);
            
            % find total amount of torque due to existing sail torque and
            % added keel torque due to keel lift
            K_torque_debt = K_torque_hydro(1) + boat_reaction(2,1);
            
            % find new required heel angle to keep balance
            boat_heel = asin(K_torque_debt/(K_span*K_mass*9.81));
            
            % find torque due to bulb at the end of the keel
            Arm_gravity = K_span*cos(boat_heel) * n_airfoil;
            K_torque_mass = cross(Arm_gravity, K_gravity);
            
            % balance everything onto the boat reaction vector
            boat_reaction(2,:) = boat_reaction(2,:) + K_torque_hydro + K_torque_mass;
            %}
        end
    end
end

