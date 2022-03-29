%%%BASE MOTOR
clear all

print = 0;
%------------------------------Simulation Parameters----------------------
extra = 0;
r2d = 180/pi;
SimTime = 5.0;
solverTimeStep = 1e-5;
CF = 1000.0;
PWMFreq = 10*1e3;
impulse_width = 1e-6;

%-------------------------------------------------------------------------
%PID PWM parameters
%{
K = 1.0;
Kp = 2.8;
Ki = 0.00;
Kd = 0.05;
%}

%creating a time vector
timeSamplingFreq = 1/solverTimeStep;
SimTimeVector = linspace(0, SimTime, SimTime*timeSamplingFreq)';


%---------------------------VoltageAmplifier-------------------------------
%------------CACL1----------------
%Experimental Data from a Data Sheet
%from the plot, I get the following values 
peak = 22.986/5; %hack 20.21 
final = 22.985/5;%hack 16.0
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
%estimate 2 (use omega_n_tp)
num2 = [ 0 0 final*omega_n_tp^2];
den2 = [1 2*zeta*omega_n_tp omega_n_tp^2];
VoltageAmplifier = tf(num2, den2);

%----------Saturation and limits-------------
Vsat = 18.0;
Isat = 10.0;
PidLim = 1024;
PidUCGain = 5/1024;
%--------------------------------------------------------------------------



%-----------------------SYSTEM MODEL---------------------------------------

%------Model Inertias and Masses----------------
JBaseMotorLoad = 0.65648373469;
JArmMotorLoad = 0.05760189627;

%------------------BASE MOTOR------------------------
Kmb = 23.4*1e-3;
Kvb = Kmb;
Lb = 0.0774e-3;
Rb = 0.212;
Jmotorb = 102*1e-7;
Jgearb = 10.7*1e-7;
Jmotorcombinedb = Jmotorb + Jgearb;%motor inertia + gear inertia 

%Calculating Motor Friction
%We know that Torque = B*v
%V = no load speed
INoLoadb = 177*1e-3;
NoLoadSpeedb = 7200/60*2*pi;
NoLoadTorqueb = Kvb * INoLoadb;
Bmotorb = NoLoadTorqueb / NoLoadSpeedb; %Bm
%Gear Friction
% Use this -> Bgear*noloadspeed = (1-efficiency)*Torque_out
Bgearb = Bmotorb;%4.58366236105*1e-2; <-Not sure about this value but this is wrong.
Bmotorcombinedb = Bmotorb + Bgearb;

GearRatiob = 111.0;
Jmotoroutb = Jmotorcombinedb*GearRatiob^2;
Bmotoroutb = Bmotorcombinedb*GearRatiob^2;

JtotalBaseMotor = Jmotoroutb + JBaseMotorLoad;
%JtotalBaseMotor = Jmotoroutb;

%-----Transfer Functions-----
Electricalb = tf([0, 1], [Lb, Rb]);
Mechanicalb = tf([0, 1], [JtotalBaseMotor, Bmotoroutb]);
BaseMotorFullModel = Electricalb * Kmb * GearRatiob * Mechanicalb/(1 + Kmb * GearRatiob * Electricalb * Kmb * GearRatiob * Mechanicalb);
%----------------------------------------------------


%----------------Sensor and Microcontroller Dynamics--------------
sensorGain = 150.0;
Hs = tf([0, 2*CF],[1, 2*CF]);


%----------------PID Controller Parameters---------------------------------
BaseMotorOpenLoop = VoltageAmplifier * BaseMotorFullModel * Hs * tf([0,1], [1,0]);

%Pid Tuning for Base Motor
Kb = 1;%15.4
Zerob = (15.5)/10;
Poleb = 2*CF;

%PID Parameters
Kpb = 2/(Zerob) - 1/Poleb ;
Kib = 1.0;
Kdb = 1/(Zerob^2) - Kpb/Poleb ;
%System Test
%{
Kpb = 1000;
Kib = 0;
Kdb = 1050;
%}

s = tf('s');
Dynb = Poleb/(Zerob^2)*((s+Zerob)^2/(s*(s+Poleb)));
%Dyn1 = Kb*(Kpb + tf([0, 1], [1, 0])*Kib + tf([2*CF, 0],[1, 2*CF])*Kdb); %Didnt Work

NewOpenLoopBase = BaseMotorOpenLoop * Dynb;
%see if zeros are correct.
if(print == 1)
    %figure(2)
    %margin(NewOpenLoopBase)
end
%Kb = 1/abs(freqresp(NewOpenLoopBase,Zerob));
Kb = 800.0;%200 20 15 8 3
NewOpenLoopBase1 = BaseMotorOpenLoop * Dynb * Kb;

SystemGb = Kb * Dynb * VoltageAmplifier * BaseMotorFullModel * 1/s;
SystemCLTFb = SystemGb/(1+SystemGb*Hs);

%StepResonses
%now generate step 
testpoints = 10000;
time = (linspace(0, 20.0, testpoints))';
input = (linspace(pi/2, pi/2, testpoints))';
CL_bsim = lsim(SystemCLTFb, input, time);
%figure(2);
hold on;
grid on;
box on;
%plot(time, CL_bsim*r2d, 'k', 'LineWidth', 3);
xlabel('Time(sec)');
ylabel('Angle(deg)');
title('Step Responses of the Closed Loop Transfer Functions MotorBase'); 
hold off;
%-----------------BASE MOTOR ENDS----------------------------------------

%----------------ARM MOTOR----------------------------------------------
Kma = 19.5*1e-3;
Kva = Kma;
La = 0.0525e-3;
Ra = 0.165;
Jmotora = 75.9*1e-7;
Jgeara = 9.961*1e-7;
Jmotorcombineda = Jmotora + Jgeara;%motor inertia + gear inertia 

%Calculating Motor Friction
%We know that Torque = B*v
%V = no load speed
INoLoada = 234*1e-3;
NoLoadSpeeda = 8630/60*2*pi;
NoLoadTorquea = Kva * INoLoada;
Bmotora = NoLoadTorquea / NoLoadSpeeda; %Bm
%Gear Friction
% Use this -> Bgear*noloadspeed = (1-efficiency)*Torque_out
Bgeara = Bmotora;%4.58366236105*1e-2; <-Not sure about this value but this is wrong.
Bmotorcombineda = Bmotora + Bgeara;

GearRatioa = 172.0;
Jmotorouta = Jmotorcombineda*GearRatioa^2;
Bmotorouta = Bmotorcombineda*GearRatioa^2;

JtotalArmMotor = Jmotorouta + JArmMotorLoad;
%-----Transfer Functions-----
Electricala = tf([0, 1], [La, Ra]);
Mechanicala = tf([0, 1], [JtotalArmMotor, Bmotorouta]);
ArmMotorFullModel = Electricala * Kma * GearRatioa * Mechanicala/(1 + Kma * GearRatioa * Electricala * Kma * GearRatioa * Mechanicala);

%----------------PID Controller Parameters---------------------------------
ArmMotorOpenLoop = VoltageAmplifier * ArmMotorFullModel * Hs * tf([0,1], [1,0]);

%Pid Tuning for Base Motor
Ka = 1;%15.4
Zeroa = (1.28e+3)/10;
Polea = 2*CF;

%PID Parameters
Kpa = 2/(Zeroa) - 1/Polea ;
Kia = 1.0;
Kda = 1/(Zeroa^2) - Kpa/Polea ;
%System Test

Kpa = 1500;
Kia = 0;
Kda = 150;


Dyna = Polea/(Zeroa^2)*((s+Zeroa)^2/(s*(s+Polea)));
%Dyn1 = Kb*(Kpb + tf([0, 1], [1, 0])*Kib + tf([2*CF, 0],[1, 2*CF])*Kdb); %Didnt Work

NewOpenLoopArm = ArmMotorOpenLoop * Dyna;
%see if zeros are correct.
if(print == 1)
    figure(3)
    margin(NewOpenLoopArm)
end
%Ka = 1/abs(freqresp(NewOpenLoopArm,Zeroa));
Ka = 1;%3000
NewOpenLoopArm1 = ArmMotorOpenLoop * Dyna * Ka;

SystemGa = Ka * Dyna * VoltageAmplifier * ArmMotorFullModel * 1/s;
SystemCLTFa = SystemGa/(1+SystemGa*Hs);

%StepResonses
%now generate step 
CL_asim = lsim(SystemCLTFa, input, time);
%figure(4);
hold on;
grid on;
box on;
%plot(time, CL_asim*r2d, 'k', 'LineWidth', 3);
xlabel('Time(sec)');
ylabel('Angle(deg)');
title('Step Responses of the Closed Loop Transfer Functions MotorBase'); 
hold off;
%--------------------------ARM MOTOR ENDS-------------------------------


