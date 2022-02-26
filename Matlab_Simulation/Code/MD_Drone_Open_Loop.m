% MD Drone - Simulation code for 4 motor micro drone
% Task: Open loop simulation
% Author: £ukasz Halbiniak

function  Done = MD_Drone_Open_Loop(A_Lin_D, B_Lin_D, C_Lin_D, D_Lin_D, Tp, T_End)
%Initial parameters
[MDRONE, GRAVITY, IXX, IYY, IZZ, D, C, Tp] = MD_Drone_Parameters()
Sample = T_End/Tp;
Fg = MDRONE * GRAVITY;
U_Input = [2*Fg/4 ;2*Fg/4;2*Fg/4;2*Fg/4];
Y_Output = zeros(6,Sample);
X_Buff = zeros(6,1);

%Simulation
for s = 1:Sample
    X_Buff = (A_Lin_D * X_Buff + B_Lin_D * U_Input) - (D_Lin_D*[1; 0; 0; 0]*Tp);
    Y_Output(4,s) = X_Buff(4,1);
    Y_Output(5,s) = X_Buff(5,1);
    Y_Output(6,s) = X_Buff(6,1);
end

%Plot
T = [Tp:Tp:T_End];
Plot = figure('Position', [10 10 650 400]);
subplot(2,1,1);
plot(T,Y_Output(4,:),'b')
grid;
title('Linear model open loop');
legend('Z')
ylabel('Position [m]');
xlabel('Time [s]');
hold off

subplot(2,1,2);
grid;
plot(T,Y_Output(5,:),'b')
hold on
plot(T,Y_Output(6,:),'g')
grid;
legend('FI','THETA')
ylabel('Degree ["]');
xlabel('Time [s]');
saveas(Plot,'MD_Open_Loop.png')
Done =1;
end
