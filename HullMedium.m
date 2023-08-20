classdef HullMedium
    
%{ 

    Fixed variables that do not change through studies
    
    Hull dampener factor
    kn to ms conversion factor

%}

    properties
        HullParameter
        DampenerFactor;
        MsToKn;
    end
    
    methods
        function obj = HullMedium(H_parameter, H_dampener, ms_to_kn)
            
            obj.HullParameter = H_parameter;
            obj.DampenerFactor = H_dampener;
            obj.MsToKn = ms_to_kn;

        end
        
        function [boat_reaction, boat_speed] = computeHullState(obj, boat_reaction, boat_speed)
            
            % current sailing condition
            current_speed_kn = obj.MsToKn * boat_speed;
            existing_hull_drag = -obj.HullParameter*current_speed_kn^2; % pointing backwards (-ve x)
            
            thrust = existing_hull_drag + boat_reaction(1,1);
            
            boat_speed_kn = sqrt(abs(thrust)/obj.HullParameter);
            new_boat_speed = boat_speed_kn/obj.MsToKn;
            
            speed_delta = new_boat_speed - boat_speed;
            
            boat_speed = boat_speed + speed_delta * obj.DampenerFactor;
            
            % we assume that the hull can counteract
            % 1. reset the net thrust due to hull drag
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

