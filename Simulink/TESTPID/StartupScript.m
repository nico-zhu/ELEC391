clear all

extra = 0;
r2d = 180/pi;

SimTime = 5.0;
CF = 100.0;
sensorGain = 150.0;

%PID Parameters
K =11.0;
Kp = 5.9;
Ki = 8.9;
Kd = 0.1;

Km = 35.6e-3;
Kv = 1./(268./60*2*pi);

L = 1.320e-3;
R = 22.8;
J = 9.545e-6;%nischay hack 2.72e-7 is only of the motor
B = 0.0007;

Amp_num = 5;
Amp_denom = 5;

impulse_width = 1e-6;

