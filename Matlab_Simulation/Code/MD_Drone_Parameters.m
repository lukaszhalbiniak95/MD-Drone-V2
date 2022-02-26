% MD Drone - Simulation code for 4 motor micro drone
% Task: Drone parameters
% Author: £ukasz Halbiniak

function [MDRONE, GRAVITY, IXX, IYY, IZZ, D, C, Tp] = MD_Drone_Parameters()
MDRONE=0.09;
GRAVITY=9.81;
IXX=0.00006825;
IYY=0.000066626;
IZZ=0.0001268;
D =0.1;
C =1;
Tp = 0.2;
end