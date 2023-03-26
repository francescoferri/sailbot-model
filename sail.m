function [S_thrust,S_torque,S_angle] = sail(rho_air, mu_air, M, W_speed, W_app_angle, S_area, S_avg_ch, S_alpha, S_span, S_mass, H_heel)

Re = rho_air*S_avg_ch*W_speed/mu_air;

% Find Closest Re that is tabulated
a=[5 2 1]; % Values of Re Tabulated, 500-200-100k
n=Re/100000;
[~,idx]=min(abs(a-n));
ReTab=100000*a(idx); % Closest Re tabulated in M

% Find Closest alpha within closest Re
alphas=zeros;
for i = 1:size(M,1)
    if (M(i,1)==ReTab)
        alphas(i,1) = M(i,2); % get alphas
    end 
end

[~,idx]=min(abs(alphas-S_alpha));
alphaTab=alphas(idx); % Closest alpha tabulated in M

% Parsing answer
alphas=zeros;
for i = 1:size(M,1)
    if (M(i,1)==ReTab && M(i,2)==alphaTab)
        cl=M(idx, 3);
        cd=M(idx, 4);
        cdp=M(idx, 5);
        cm=M(idx, 6);
    end
end

Ls = 0.5*cl*S_area*W_speed^2*rho_air; % sail lift
Ds = 0.5*cd*S_area*W_speed^2*rho_air; % sail drag

S_angle=W_app_angle-deg2rad(alphaTab);

L_Ls = Ls*cos(H_heel)*sin(W_app_angle); % Longitudinal Lift
L_Ds = Ds*cos(H_heel)*cos(W_app_angle); % Longitudinal Drag
S_thrust = L_Ls - L_Ds; % Sail Thrust

T_Ls = Ls*cos(H_heel)*cos(W_app_angle); % Transverse Lift
T_Ds = Ds*cos(H_heel)*sin(W_app_angle); % Transverse Drag
Hs = T_Ls - T_Ds; % Sail Heeling Force

S_torque = Hs*(S_span*cos(H_heel))/2 + S_mass*9.81*(S_span*sin(H_heel))/2;

end