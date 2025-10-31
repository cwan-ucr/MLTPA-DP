function Energy = rta_battery(V, A)

% m = 1521; % mass, kg
% g = 9.81; % graviational acceleration, m/s^2
% theta = 0; % road degree, rad
% Cr = 1.75; % rolling resistance parameter 1
% c1 = 0.0328; % rolling resistance parameter 2
% c2 = 4.575; % rolling resistance parameter 3
% rho = 1.2256; % air density
% Af = 2.3316; % front area
% Cd = 0.28; % aerodynamic drag coefficient
% 
% nDL = 0.92; % driverline
% nEM = 0.91; % motor
% nBAT = 0.90; % battery
% alpha = 0.0411; % shape parameter
% 
% F = m * A + m * g * cos(theta) * (Cr / 1000) * (c1*V + c2) + ...
%             0.5 * rho * Af * Cd * V.^2 + m * g * sin(theta);
% Pw = F.*V;
% 
% index = Pw < 0;
% nRB = min(1, exp(-alpha ./ abs(A)));
% 
% Pt = Pw / (nDL * nEM * nBAT);
% Pc = Pw .* (nDL * nEM * nBAT * nRB);
% 
% Energy = ~index.*Pt + index.*Pc;
% Energy = Energy/1000; % Wh/s

% Calculate energy consumption of a vehicle with speed v and acceleration a
% Based on: Wu, et al. (2015) http://dx.doi.org/10.1016/j.trd.2014.10.007
%
% Inputs:
%   v - vehicle speed in m/s
%   a - vehicle acceleration in m/s^2
%
% Output:
%   P_wh_s - Power consumption in Wh/s
% 
% % --- 参数定义 (Parameter Definitions) ---
frl = 0.006;      % 滚动阻力系数 (Rolling resistance coefficient)
k = 1.30;         % 风阻系数 (Air drag coefficient)
K = 10.08;        % 空气阻力系数 (Air resistance coefficient)
r = 0.11;         % 电阻 (Resistance, ohms)
R = 0.5;          % 轮胎半径 m (Tire radius, m)
g = 9.81;         % 重力加速度 m/s^2 (Gravitational acceleration)
M_e = 1266;       % 车辆质量 kg (Vehicle mass, kg)
G = 0;            % 坡度 (Grade, dimensionless)
efficiency = 0.95; % 电池效率 (Battery efficiency for regenerative braking)

% --- 计算总力 F (Calculate total force F) ---
% F = Inertial Force + Aerodynamic Drag + Rolling Resistance + Gradient Force
F = M_e * A + k * (V.^2) + frl * M_e * g + M_e * g * G;

% --- 计算功率 P (Calculate Power P) ---
% The formula calculates power based on whether the total force F is
% positive (propulsion) or negative (braking/coasting).
if F < 0
    % Negative F implies braking (regenerative braking case)
    % The efficiency factor is applied to the inertial component during regeneration.
    P = r * (R^2) * (F.^2) / (K^2) + V .* (M_e * A * efficiency + k * (V.^2) + frl * M_e * g + M_e * g * G);
else
    % Positive F implies propulsion
    P = r * (R^2) * (F.^2) / (K^2) + V .* F;
end

% --- 转换单位为 Wh/s (Convert units to Wh/s) ---
% The calculated power P is in Watts (Joules/second). 
% To convert to Watt-hours per second, we divide by 3600.
Energy = P / 3600; 

% parameter ARRB
% alpha = 0.666;
% beta1 = 0.0717;
% beta2 = 0.0344;
% b1 = 0.269;
% b2 = 0.0171;
% b3 = 0.000672;
% m = 1600;
% g = 9.81;
% G = 0;
% 
% 
% Rt = b1+b2*V+b3*(V.^2)+m*A/1000+m*g*G/100000;
% index_1 = (Rt>0&A<=0);
% index_2 = (Rt>0&A>0);
% Fuel1 = beta1*Rt.*V;
% Fuel2 = beta1*Rt.*V+beta2*m*V.*A.^2/1000;
% Fuel = index_1.*Fuel1 + index_2.*Fuel2;
% Energy = alpha + max(0, Fuel);



