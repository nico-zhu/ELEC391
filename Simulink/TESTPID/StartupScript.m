%%%BASE MOTOR
clear all

extra = 0;
r2d = 180/pi;

SimTime = 5.0;
solverTimeStep = 1e-5;
CF = 1000.0;
sensorGain = 150.0;
PWMFreq = 10*1e3;

%PID Parameters
K = 1.0;
Kp = 2000.0;
Ki = 0;
Kd = 0.0;

%PID PWM parameters

K = 1.0;
Kp = 2.8;
Ki = 0.00;
Kd = 0.05;
%}

Km = 23.4*1e-3;
Kv = Km;

L = 0.0774e-3;
R = 0.212;
J = (102+10.7)*1e-7;%nischay hack 2.72e-7 is only of the motor

Amp_num = 5;
Amp_denom = 5;

%We know that Torque = B*v
%V = no load speed
INoLoad = 177*1e-3;
NoLoadSpeed = 7200/60*2*pi;
NoLoadTorque = Kv * INoLoad;
B = NoLoadTorque / NoLoadSpeed; %Bm

GearRatio = 111.0;
J = J*GearRatio^2;
B = B*GearRatio^2;

impulse_width = 1e-6;

%creating a time vector
timeSamplingFreq = 1/solverTimeStep;
SimTimeVector = linspace(0, SimTime, SimTime*timeSamplingFreq)';
%------------VoltageAmplifier----------
%------------CACL1----------------
%Experimental Data from a Data Sheet
%from the plot, I get the following values 
peak = 17.986/5; %hack 20.21 
final = 17.985/5;%hack 16.0
Overshoot_val = peak - final;
RiseTime = 9.5897*1e-6; %hack 18ms
PeakTime = 9.5899*1e-6; %hack 27ms
SettleTime = 10.0*1e-6; %hack 78ms
Overshoot_pct = (peak-final)/final*100; %percentage

%-----------CALC2-----------------
%we can estimate zeta from %overshoot as
%percent_OS = e^(-zeta*pi/sqrt(1-zeta^2))
%so zeta = sqrt(ln^2(OS/FV)/(pi^2+ln^2(OS/Fv))
zeta = sqrt(log(Overshoot_val/final)^2/(pi^2+log(Overshoot_val/final)^2));
%to calculate wn = 4/(Zeta*SettleTime)
omega_n = 4/(zeta*SettleTime*1);

%-----------CALC3----------------
%Wn using peaktime = pi/(beta*Tpeak)
beta = sqrt(1- zeta^2);
omega_n_tp = pi/(1*PeakTime*beta);

%----------CALC4------------------
%Omega_n using RiseTime = atan(beta/zeta)/(beta*Trise)
omega_n_tr = (pi-atan(beta/zeta))/(beta*RiseTime);

%---------Making Tranfer Function---
%based on the notes, the TF of of second order system is
% TF(O2) = wn^2/(s^2 + 2*zeta*wn*s + wn^2)
%estimate 1 (use omega_n)
num1 = [ 0 0 final*omega_n^2];
den1 = [1 2*zeta*omega_n omega_n^2];
tf1 = tf(num1, den1);

%estimate 2 (use omega_n_tp)
num2 = [ 0 0 final*omega_n_tp^2];
den2 = [1 2*zeta*omega_n_tp omega_n_tp^2];
VoltageAmplifier = tf(num2, den2);

%estimate 1 (use omega_n_tr)
num3 = [ 0 0 final*omega_n_tr^2];
den3 = [1 2*zeta*omega_n_tr omega_n_tr^2];
tf3 = tf(num3, den3);
%------------------------------------------------------

