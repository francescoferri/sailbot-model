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
        
        function [boat_reaction, boat_heel, at_stall] = computeKeelState(obj, W_speed, W_angle, boat_reaction, boat_speed, boat_heel, K_span, K_avg_ch, K_mass, K_airfoil)
            
            %% initialize

            K_area = K_span * K_avg_ch; % [m2] assumed to be square
            K_gravity = [0 0 -K_mass*9.81]; % keel force of gravity
            K_force_hydro = [0 0 0];
            K_alpha = 0;

            water_speed = norm(boat_speed);
            reynolds_number = obj.FluidDensity * K_avg_ch * water_speed/obj.FluidViscosity;
            
            % airfoil plane (keel's direction)
            n_keelfoil = [0 sin(boat_heel) -cos(boat_heel)];

            % water direction
            water = [-boat_speed(1) -boat_speed(2) 0];
            u_water = water/norm(water);

            % vertical vecotr
            vertical = [0 0 1];

            % normal to water plane
            n_water = cross(u_water, vertical);

            % using the two planes compute direction of drag
            drag_direction = cross(n_water, n_keelfoil);

            % using the drag direction and keel direction we find lift
            lift_direction = cross(drag_direction, n_keelfoil);
            
            %% increase alpha untill we reach equilibrium
            at_stall=0;

            while( (abs(K_force_hydro(1,2)) < abs(boat_reaction(1,2))) && at_stall==false)

                % if we stall, we stop increasing alpha and move on with
                % the hydro force we have available, by not updating coefficients 
                K_alpha = K_alpha + 0.01;

                % find matching coeficient
                [cl, cd, cdp, cm, at_stall] = findAirfoilCoeff(reynolds_number, K_alpha, K_airfoil);

                L = 0.5*cl*K_area*water_speed^2*obj.FluidDensity; % keel lift
                D = 0.5*cd*K_area*water_speed^2*obj.FluidDensity; % keel drag
    
                Lift = lift_direction*L;
                Drag = drag_direction*D;
    
                K_force_hydro = Lift + Drag;

            end

            % record force changes and move on
            K_force = K_force_hydro + K_gravity;
            boat_reaction(1,:) = boat_reaction(1,:) + K_force;

            %% find new heel angle

            % find total keel torque based on previous heel angle
            r_hydro = K_span/2;
            arm_hydro = n_keelfoil*r_hydro;
            K_torque_hydro = cross(arm_hydro, K_force_hydro);

            r_gravity = K_span;
            arm_gravity = n_keelfoil*r_gravity;
            K_torque_gravity = cross(arm_gravity, K_gravity);

            K_torque = K_torque_gravity + K_torque_hydro;

            % sum y components
            K_torque_debt = boat_reaction(2,1) + K_torque(1,2);

            boat_heel_new = -asin(K_torque_debt/(K_span*K_mass*9.81));
            heel_delta = boat_heel_new - boat_heel;
            boat_heel = boat_heel + heel_delta;

            %{

GRAVEYARD

            

%% New Heel Angle

            % determine new heel angle by solving force balance based on
            % previously calculated forces.
            syms new_boat_heel_sym;
            new_n_keelfoil_sym = [0 sin(new_boat_heel_sym) -cos(new_boat_heel_sym)];

            % Boat heel must be defined less than 90 to get one solution
            constraints1 = new_boat_heel_sym <= deg2rad(90);
            constraints2 = new_boat_heel_sym >= deg2rad(-90);

            K_torque_x = -boat_reaction(2,1);

            % Torque from Hydro (symbolic)
            % Location of applied hydro force (switch with COP in the future)
            R_Hydro = K_span/2;
            arm_Hydro = R_Hydro*new_n_keelfoil_sym;
            K_torque_hydro = cross(arm_Hydro,K_force_hydro);

            %Torque from Gravity (symbolic)
            R_Gravity = K_span; % approximated as end of keel. Add COG later
            arm_Gravity = R_Gravity*new_n_keelfoil_sym;
            K_torque_gravity = cross(arm_Gravity,K_gravity);

            eqn = K_torque_x == K_torque_gravity(1) + K_torque_hydro(1);
            new_boat_heel = double(solve(eqn,constraints1,constraints2, ...
                new_boat_heel_sym,"Real",true));

            if (isempty(new_boat_heel))
                error 'Boat flipped'
            end
            
            %% Keel Torques
            new_n_keelfoil = [0 sin(new_boat_heel) -cos(new_boat_heel)];

            K_torque = cross(R_Hydro*new_n_keelfoil, K_force_hydro) + ...
            cross(R_Gravity*new_n_keelfoil,K_gravity);

            boat_reaction(2,:) = boat_reaction(2,:) + K_torque;

            % change to new heel
            boat_heel = new_boat_heel;

% yes we are able to generate enough lift with the keel, find the hydro force component at equilibrium
                syms cl_over_cd_sym;

                L = 0.5*cl_over_cd_sym*K_area*water_speed^2*obj.FluidDensity; % keel lift
                D = 0.5*1*K_area*water_speed^2*obj.FluidDensity; % keel drag
    
                Lift = lift_direction*L;
                Drag = drag_direction*D;
    
                force_hydro_y = Lift + Drag;

                eqn = force_hydro_y(2) == -boat_reaction(1,2);

                cl_over_cd = double(solve(eqn,cl_over_cd_sym,"Real",true));
                
                [cl, cd] = findCdClwithRatio(reynolds_number, cl_over_cd, K_airfoil);

                L = 0.5*cl*K_area*water_speed^2*obj.FluidDensity; % keel lift
                D = 0.5*cd*K_area*water_speed^2*obj.FluidDensity; % keel drag
    
                Lift = lift_direction*L;
                Drag = drag_direction*D;
    
                K_force_hydro = Lift + Drag;

                % record force changes and move on
                K_force = K_force_hydro + K_gravity;
                boat_reaction(1,:) = boat_reaction(1,:) + K_force;

%% find greatest lift available
            K_alpha = 0;
            at_stall=0;

            while(at_stall==false)

                K_alpha = K_alpha + 0.1;

                % find matching coeficient
                [cl, cd, cdp, cm, at_stall] = findAirfoilCoeff(reynolds_number, K_alpha, K_airfoil);
            end
            
            L = 0.5*cl*K_area*water_speed^2*obj.FluidDensity; % keel lift
            D = 0.5*cd*K_area*water_speed^2*obj.FluidDensity; % keel drag

            Lift = lift_direction*L;
            Drag = drag_direction*D;

            K_force_hydro = Lift + Drag;

            %% are we able to counteract the sail?
            if ( abs(K_force_hydro(1,2)) <= abs(boat_reaction(1,2)))

                % no we are not able to, record force changes and move on
                K_force = K_force_hydro + K_gravity;
                boat_reaction(1,:) = boat_reaction(1,:) + K_force;

            else
                
                while( (abs(K_force_hydro(1,2)) < abs(boat_reaction(1,2))) && at_stall==false)

                    K_alpha = K_alpha + 0.01;
    
                    % find matching coeficient
                    [cl, cd, cdp, cm, at_stall] = findAirfoilCoeff(reynolds_number, K_alpha, K_airfoil);

                    L = 0.5*cl*K_area*water_speed^2*obj.FluidDensity; % keel lift
                    D = 0.5*cd*K_area*water_speed^2*obj.FluidDensity; % keel drag
        
                    Lift = lift_direction*L;
                    Drag = drag_direction*D;
        
                    K_force_hydro = Lift + Drag;
                    
                end

                % record force changes and move on
                K_force = K_force_hydro + K_gravity;
                boat_reaction(1,:) = boat_reaction(1,:) + K_force;

            end

            %}

        end
    end
end

