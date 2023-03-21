%--------------------------------------------------------------------------
%
%                Model based design for Self-Balancing Robot
%
%--------------------------------------------------------------------------
% Authors: Arshad S. 
%          Dhinesh R.
%
% email: arsh402@gmail.com
%        dhineshrajasekaran@gmail.com
%
% November 2022; Last revision: November-2022
%--------------------------------------------------------------------------
%
% Description: This is main script to run the simulink model -
% 'SelfBalRobot_MBD'
%
%--------------------------------------------------------------------------
%
% References:
% Paper: Model-based Design for a Self-balancing Robot using the 
% Arduino Micro-controller Board
% 978-1-7281-3998-2/19/$31.00 ©2019 IEEE
%
%--------------------------------------------------------------------------
%% Initializing the workspace
clc
clear
close all

format short
%% Define the parameters

% Physical parameters of the self-balancing robot
mw = 405 * 10^-3 ; % Wheel mass (kg)
mb = 640 * 10^-3;% Body mass (kg)
m = 1045 * 10^-3; % Total mass (kg)
Iw = 390*10^(-6); % Wheel inertia (kg.m2)
Ib = 267 * 10^(-3); % Body inertia (kg.m2)
rWheel = (4.55) * 10^-2; % Radius of wheel (m)
g = 9.81; % Gavity (ms-2)
lCOM = 17.73 * 10^-2; % Distance COM and wheel (m)
betaY = 0.01 ;% Rolling damping ratio (N.m / (rad/s))
betaM = 0.01 ;% Friction damping ratio (N.m/ (rad/s))

% Motor parameters of the self-balancing robot
N = 1; % 52.734/1;
P = 6;
km = 0.22 ;%* 0.1019716213; % Torque constant (kg f/A)
kb = 0.122; % Voltage constant (V/(rad/s))
Im = 26.7*10^(-3); % Total Inertia (V/(rad/s))
c = 0.25 ;%* 0.1019716213; % Damping coefficient (kg/(rad/s))
La = 12*10^-3; % Armature Inductance (H)
Ra = 3.3; % Armature Resistance (ohm)


initial_theta = deg2rad(-5);
Ts = 0.01;
%% 
num1 = km
den1 = Im*La
den2 = ((c * La) + (Im * Ra))
den3 = ((c * Ra) + (kb * km))


%% Define the matrices

E = [(Iw + (mw + mb) * rWheel^2),     (mb * rWheel * lCOM);
     (mb * rWheel * lCOM),            (Ib + mb * lCOM^2)];

F = [(betaY + betaM),    (-betaM);
     (-betaM),           (betaM)];

G = [0; (-mb * g * lCOM)];
H = [1; -1];

% % First define the matrices to use their elements easily
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0];

A(3:4,2) = -inv(E)*G;
A(3:4,3:4) = -inv(E)*F;
disp('A');
disp(A);
% 
B = [0; 0; 0; 0];
B(3:4,1) = -inv(E)*H;
disp('B');
disp(B);

% Output

C = [rWheel, 0, 0, 0;
     0,      1, 0, 0]; % 'x = rWheel*Phi' & 'Theta' as outputs
% C = [0, 1, 0, 0]; % q_1 as output
disp('C');
disp(C);

D = 0;

%% Build System

sys = ss(A,B,C,D)

% x = [Phi Theta PhiDot ThetaDot]
x0 = [0, 5.72958*pi/180, 0, 0];
% x0 = [0, 0, 0, 0];
% Short analysis of the system - System is stable or not
eig = eig(A)
poles = pole(sys)
Sc = ctrb(sys)
So = obsv(sys)
rnk_Sc = rank(Sc)
rnk_So = rank(So)   

% rlocus(sys)

% %% Controller
% % 
% % des_poles = [-1.4; -1.4; -1.4; -1.4];  % shifts all poles to -1.4
% % K = acker(A,B, des_poles); % Controller is given by command acker or place
% 
Q = eye(4)
R = 1
K_lqr = lqr(A,B,Q,R)
K_lqr_given = [1 67.5 0.957 13.8];
% 
