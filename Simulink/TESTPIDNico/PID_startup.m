%simulation time
simtime = 15;

%control frequency
CF = 500;
Sg = 1;

% Terminal Resistance
R = 2;
% Terminal inductance
L = 1.8e-3;
% Gear ratio
a = 100;
% Moment of Inertia
Jm = 1e-7;
Jp = 1e-8;
Jl = 5e-7;
Jg = 1e-8;
JLS = a^2*(Jm+Jp) + Jg + Jl;
% Friction Constant
Bm = 1e-8;
Bt = 1e-8;
Bg = 1e-8;
BLS = a^2*(Bm + Bt) + Bg;

%PID constants
K = 1;
Kp = 2; 
Ki = 2;
Kd = 5;

% Km: Torque Constant
Km = 6.95e-3;