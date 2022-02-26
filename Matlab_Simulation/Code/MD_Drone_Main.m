% MD Drone - Simulation code for 4 motor micro drone
% Tasks:
% - Nonlinear model
% - Linear model
% - Discreditation
% - Simulation drone with LQR
% - Saving results
% Author: £ukasz Halbiniak
clear all; clc;

% Linearization algorithm
% A_lin, B_Lin, C_Lin, D_Lin - Continous drone model
% A_lin_D, B_Lin_D, C_Lin_D, D_Lin_D - Discrete drone model
[A_Lin_D, B_Lin_D, C_Lin_D, D_Lin_D, A_Lin, B_Lin, C_Lin, D_Lin, Tp] = MD_Drone_Linearization();

%Open loop simulation - Discrete drone model
Done_open_loop = MD_Drone_Open_Loop(A_Lin_D, B_Lin_D, C_Lin_D, D_Lin_D, Tp, 15)

%LQR regulator simulation - Discrete drone model
Done_LQR = MD_Drone_LQR(A_Lin_D, B_Lin_D, C_Lin_D, D_Lin_D, A_Lin, B_Lin, C_Lin, D_Lin, Tp, 80)

