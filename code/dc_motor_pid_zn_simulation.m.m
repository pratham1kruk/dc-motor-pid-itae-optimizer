% DC Motor + PID Controller Simulation
clc; clear; close all;

% --- Motor Parameters ---
R = 1;          % Armature Resistance (ohm)
L = 0.5;        % Armature Inductance (H)
J = 0.01;       % Moment of Inertia (kg.m^2)
B = 0.1;        % Friction coefficient (N.m.s)
Kt = 0.01;      % Torque constant
Ke = 0.01;      % Back emf constant

% --- Transfer Function of DC Motor ---
num = [Kt];
den = [(L*J) (L*B + R*J) (R*B + Ke*Kt)];
G = tf(num, den);
disp('Transfer Function of DC Motor:');
G

% --- Step Response without controller ---
figure;
step(G)
title('Open Loop Step Response of DC Motor');
grid on;

% --- Ziegler–Nichols PID Tuning ---
% Find ultimate gain Ku and period Pu (manual)
Ku = 70;   % assume obtained experimentally
Pu = 0.8;  % seconds

Kp = 0.6 * Ku;
Ki = 2 * Kp / Pu;
Kd = Kp * Pu / 8;

PID = pid(Kp, Ki, Kd);

% --- Closed Loop System ---
sys_cl = feedback(PID*G, 1);

% --- Step Response with PID ---
figure;
step(sys_cl)
title('Closed Loop Step Response with Z-N PID Controller');
grid on;

% --- Performance Metrics ---
info = stepinfo(sys_cl)
