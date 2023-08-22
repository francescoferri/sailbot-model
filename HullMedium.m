classdef HullMedium
    
%{ 

    Fixed variables that do not change through studies
    
    Hull dampener factor
    kn to ms conversion factor

%}

    properties
        HullParameter
        HullMaxSpeed
        DampenerFactor;
        MsToKn;
    end
    
    methods
        function obj = HullMedium(H_max_speed, H_parameter, H_dampener, ms_to_kn)
            
            obj.HullMaxSpeed = H_max_speed;
            obj.HullParameter = H_parameter;
            obj.DampenerFactor = H_dampener;
            obj.MsToKn = ms_to_kn;

        end
        
        function [boat_reaction, boat_speed] = computeHullState(obj, boat_reaction, boat_speed)
            
            %% find new boat speed
            speed = norm(boat_speed); % speed magnitude [ms]
            u_speed = boat_speed/speed; % speed direction (X-Y plane)

            % current sailing condition
            current_speed_kn = obj.MsToKn * speed;
            existing_hull_drag = -obj.HullParameter*current_speed_kn^2; % pointing backwards (-ve x)

            thrust = existing_hull_drag + boat_reaction(1,1);

            speed_kn = sqrt(abs(thrust)/obj.HullParameter);
            new_speed = speed_kn/obj.MsToKn;

            speed_delta = new_speed - speed;

            speed = speed + speed_delta * obj.DampenerFactor;

            %% check we are not planing
            if (speed > obj.HullMaxSpeed)
                speed = obj.HullMaxSpeed;
            end

            %% re-assign direction and leave

            boat_speed = speed * u_speed;


            %% reset boat reaction vector
            % we assume that the hull can counteract
            % 1. the net thrust (thrust = drag --> net = 0)
            boat_reaction(1,1) = 0;
            % 2. force of gravity (with boiancy)
            boat_reaction(1,3) = 0;
            % 3. pitching moment with the bow
            boat_reaction(2,2) = 0;
            % 4. yawing moment with the rudder
            boat_reaction(2,3) = 0;

        end
    end
end

