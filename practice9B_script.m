%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          MR3003. Autonomy of Unmanned Aerial Vehicles
%            Practice 9 - State Feedback Gain Matrix
%                 Part B. Regulator Control Design
%             Dr. Carlos Sotelo - Dr. David Sotelo
%                 Carlos Hernán Auquilla Larriva
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Last update: September 8th 2024

%% Step 1: Load Qube Servo 3 Parameters
clear;clc;close all;
pendParam; %load pendulum parameters
%Pendulum configuration
d = -1; %1 for crane configuration, -1 for inverted configuration

%%State Space Representation matrices
A = (1/Jt)*[0       0           Jt                   0;
            0       0            0                  Jt;
            0 mp^2*l^2*r*g -Jp*(br+km^2/Rm)     d*mp*l*r*bp;
            0 -d*mp*g*l*Jr d*mp*l*r*(br+km^2/Rm) -Jp*bp;];

B = [   0;
        0;
    km*Jp/(Rm*Jt);
    -d*mp*r*l*km/(Rm*Jt);];

C = [1 0 0 0;
     0 1 0 0];

D = [0;0];

sys = ss(A,B,C,D);

%Compute desired poles
MP=6.81/100;%Overshoot
tss=1.54;%sec
desPoles_2nd = calculatePoles(MP,tss);%Calculate 2nd order desired poles
extra_poles=[-40, -45];%Non-dominant poles
P = [desPoles_2nd, extra_poles];

%Compute controllability matrix
CO = ctrb(A,B);
if rank(CO)==length(A)
    disp("The system is controllable");
else
    disp("The system is not controllable");
end

%Compute the state feedback matrix K
%Matlab Method
K = place(A,B,P);
%Transformation Matrix T Method
[K_transf,~,~] = transformT(A,B,P);

%Companion Matrix Method
k_comp = companionM(A,B,P);


%% Plot the response
% Load data
load("QuanserLabs_Practice9_PartB_Data.mat");
t = data(1,:);
arm_desired = data(3,:);
arm_measure = data(4,:);
pend_desired = data(5,:);
pend_measured = data(6,:);
% Plot response 
subplot(2,1,1); 
plot(t,arm_desired,'r--',t,arm_measure,'g-'); 
ylabel('Rotary Arm (rad)'); 
legend('Desired', 'Measured');
subplot(2,1,2); 
plot(t,pend_desired,'r--',t,pend_measured,'g-'); 
ylabel('Pendulum (rad)'); 
xlabel('Time (s)'); 
legend('Desired', 'Measured');

% Performance indexes
A_mp=0.503146-0.454058;
B_mp=0.454058;
MP = A_mp/B_mp;
settlingTime = 1.304; %sec
