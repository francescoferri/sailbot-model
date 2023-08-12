classdef SailMedium
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        FluidDensity;
        FluidViscosity;
        M;
        Area;
        AverageChord;
        AngleOfAttack;
        Span;
        Mass;
        WindSpeed;
    end
    
    methods
        function obj = SailMedium(rho_air, mu_air, M, W_speed, S_area, S_avg_ch, S_alpha, S_span, S_mass)
            %SAILMEDIUM Construct an instance of this class
            
            obj.FluidDensity = rho_air;
            obj.FluidViscosity = mu_air;
            obj.M = M;
            obj.WindSpeed = W_speed;
            obj.Area = S_area;
            obj.AverageChord = S_avg_ch;
            obj.AngleOfAttack = S_alpha;
            obj.Span = S_span;
            obj.Mass = S_mass;
        end
        
        function [S_thrust,S_torque,S_angle] = computeSailState(obj, W_app_angle, H_heel)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            Re = obj.FluidDensity*obj.AverageChord*obj.WindSpeed/obj.FluidViscosity;

            % Find Closest Re that is tabulated
            a=[5 2 1]; % Values of Re Tabulated, 500-200-100k
            n=Re/100000;
            [~,idx]=min(abs(a-n));
            ReTab=100000*a(idx); % Closest Re tabulated in M
            
            % Find Closest alpha within closest Re
            alphas=zeros;
            for i = 1:size(obj.M,1)
                if (obj.M(i,1)==ReTab)
                    alphas(i,1) = obj.M(i,2); % get alphas
                end 
            end
            
            [~,idx]=min(abs(alphas-obj.AngleOfAttack));
            alphaTab=alphas(idx); % Closest alpha tabulated in M
            
            % Parsing answer
            alphas=zeros;
            for i = 1:size(obj.M,1)
                if (obj.M(i,1)==ReTab && obj.M(i,2)==alphaTab)
                    cl=obj.M(idx, 3);
                    cd=obj.M(idx, 4);
                    cdp=obj.M(idx, 5);
                    cm=obj.M(idx, 6);
                end
            end
            
            Ls = 0.5*cl*obj.Area*obj.WindSpeed^2*obj.FluidDensity; % sail lift
            Ds = 0.5*cd*obj.Area*obj.WindSpeed^2*obj.FluidDensity; % sail drag
            
            S_angle=W_app_angle-deg2rad(alphaTab);
            
            L_Ls = Ls*cos(H_heel)*sin(W_app_angle); % Longitudinal Lift
            L_Ds = Ds*cos(H_heel)*cos(W_app_angle); % Longitudinal Drag
            S_thrust = L_Ls - L_Ds; % Sail Thrust
            
            T_Ls = Ls*cos(H_heel)*cos(W_app_angle); % Transverse Lift
            T_Ds = Ds*cos(H_heel)*sin(W_app_angle); % Transverse Drag
            Hs = T_Ls - T_Ds; % Sail Heeling Force
            
            S_torque = Hs*(obj.Span*cos(H_heel))/2 + obj.Mass*9.81*(obj.Span*sin(H_heel))/2;
        end
    end
end

