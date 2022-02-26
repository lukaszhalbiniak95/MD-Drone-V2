% MD Drone - Simulation code for 4 motor micro drone
% Task: Linearization
% Author: £ukasz Halbiniak

function [A_Lin_D, B_Lin_D, C_Lin_D, D_Lin_D, A_Lin, B_Lin, C_Lin, D_Lin, Tp] = MD_Drone_Linearization()

% State vector -> X_State
% W - Linear velocity 
% P Q R - Angular velocity 
% Z - Drone position
% FI THETA PSI - Drone rotation
syms W P Q R Z FI THETA PSI;
X_State = [P Q R FI THETA PSI];

% Output vector -> U_State
% F1 F2 F3 F4 - Motor forces
syms F1 F2 F3 F4;
U_State = [F1 F2 F3 F4];

% Drone angular velocity equations
% T matrix
T = [1, (tan(THETA)*sin(FI)), (tan(THETA)*cos(FI)); 
    0, cos(FI), (-sin(FI))
    0, (sin(FI)/cos(THETA)), (cos(FI)/cos(THETA))];

% Angular velocity equations
OmegaB = [P;Q;R];
Angular_Velocity = T*OmegaB

% Drone parameters
% D - Distance beetween motor and center of mass
% C - Scalling factor
% IXX IYY IZZ - Drone moments of inertia
syms IXX IYY IZZ;
syms D C;
MXX = D *(F2 - F4);
MYY = D *(F1 - F3);
MZZ = C *(-F1 + F2 - F3 + F4);

% Drone angular acceleration 
Angular_Accel = [(MXX/IXX)-((IZZ-IYY)*Q*R)/IXX;(MYY/IYY)-((IXX-IZZ)*P*R)/IYY;(MZZ/IZZ)-((IYY-IXX)*P*Q)/IZZ;]

% Linearization
Matrix_All =  [Angular_Accel; Angular_Velocity]
Matrix_A = jacobian(Matrix_All,X_State)
Matrix_B = jacobian(Matrix_All,U_State)

% Trimming
P= 0;Q= 0;R= 0;FI = 0;THETA = 0;PSI = 0;

% Real drone parameters
[MDRONE, GRAVITY, IXX, IYY, IZZ, D, C, Tp] = MD_Drone_Parameters();

% Evaluation matrix - Final continous model
A_Lin = eval(Matrix_A);
B_Lin = eval(Matrix_B);

%Adding Z position
A_Lin = [0 0 0 0 0 0; [[ [0; 0] A_Lin(1:2,[1:2, [4:5]])] [0; 0] ]; 1 0 0 0 0 0; [[ [ 0; 0] A_Lin(4:5,[1:2, [4:5]])] [0; 0] ] ]
B_Lin = [1/MDRONE 1/MDRONE 1/MDRONE 1/MDRONE; B_Lin(1:2,:); 0 0 0 0; B_Lin(4:5,:) ]
Fg = MDRONE*GRAVITY;
D_Lin = zeros (6,4);
D_Lin(1,1) = GRAVITY;

% Dicreditation
C_Lin = diag([1 0 0 0 1 1])
%C_Lin = diag([1 1 1 1 1 1 ])
A_Lin_D = diag([1 1 1 1 1 1]) + (A_Lin*Tp);
B_Lin_D = B_Lin * Tp
C_Lin_D = C_Lin
D_Lin_D = D_Lin
end