% MD Drone - Simulation code for 4 motor micro drone
% Task: LQR simulation
% Author: £ukasz Halbiniak

function  Done = MD_Drone_LQR(A_Lin_D, B_Lin_D, C_Lin_D, D_Lin_D, A_Lin, B_Lin, C_Lin, D_Lin, Tp, T_End)
%Calculation LQR parameters
Sys_SS = ss(A_Lin_D,B_Lin_D,diag([1 1 1 1 1 1 ]), zeros(6,4),Tp)
Con = ctrb(Sys_SS);
Contrl = rank(Con)
Obs = obsv(Sys_SS);
Observ = rank(Obs)
Q = (C_Lin_D'*C_Lin_D)*1;
Q(1,1)=1;
Q(4,4)=1;
Q(5,5)=6;
Q(6,6)=6;
R = diag([1 1 1 1])*1;
K_LQR = dlqr(A_Lin_D,B_Lin_D,Q,R);
Ac = [(A_Lin_D-B_Lin_D*K_LQR)];
poles = eig(Ac)

%Initial parameters
Sample = T_End/Tp;
[MDRONE, GRAVITY, IXX, IYY, IZZ, D, C, Tp] = MD_Drone_Parameters();
[X_Reg] = MD_Drone_Trajectory(Sample);
Fg = MDRONE * GRAVITY;
Motor_Max = (Fg*1.2)/4;
Y_Output_Buff = zeros(6,1);
Y_Output_Noise_Buff = zeros(6,1);
Y_Output_Filtered_Buff = zeros(6,1);
U_Input_Buff = zeros(4,1);
F_Input_Buff = zeros(4,1);
Y_Output = zeros(3,Sample);
Y_Output_Noise = zeros(3,Sample);
Y_Output_Filtered = zeros(3,Sample);
U_Input = zeros(4,Sample);
F_Input = zeros(4,Sample);
X_Buff = zeros(6,1);

%Kalman filter design
V = diag(ones(4,1))*Tp;
V(1,1)=0.002;
V(2,2)=0.002;
V(3,3)=0.002;
V(4,4)=0.002;
W = diag(ones(6,1));
W(1,1)=1000;
W(2,2)=1;
W(3,3)=1;
W(4,4)=2;
W(5,5)=3;
W(6,6)=3;

[kest,L,P]  = kalman(Sys_SS,V,W);
X_Prior_L = zeros(6,1);

%Simulation
for s = 1:Sample
    
    %Model simulation
    X_Buff = (A_Lin_D * X_Buff + B_Lin_D * U_Input_Buff) - (D_Lin_D*[1; 0; 0; 0]*Tp);

    %Adding grafity force
    if (X_Buff(4,1)<=0) % Ground
        X_Buff(4,1)=0;
    end
    Y_Output_Buff = C_Lin_D * X_Buff;
    Y_Output_Buff(4,1) = X_Buff(4,1);
    
    %Adding noise
    Y_Output_Noise_Buff(1,1) = Y_Output_Buff(1,1)+(randn()*0.2);
    Y_Output_Noise_Buff(5,1) = Y_Output_Buff(5,1)+(randn()*2);
    Y_Output_Noise_Buff(6,1) = Y_Output_Buff(6,1)+(randn()*2);
    
    %Kalman filter and filtering
    X_Prior = A_Lin_D *X_Prior_L + B_Lin_D * U_Input_Buff - (D_Lin_D*[1; 0; 0; 0]*Tp) + L *(Y_Output_Noise_Buff-C_Lin_D*X_Prior_L );
    X_Prior_L = X_Prior;
    Y_Output_Filtered_Buff = X_Prior;
    if (Y_Output_Filtered_Buff(4,1)<=0) % Ground
        %Y_Output_Filtered_Buff(5,1)=0;
    end
    %Controlling
    U_Input_Buff = -(K_LQR * (Y_Output_Filtered_Buff-X_Reg(:,s)));
    %U_Input_Buff = -(K_LQR * (Y_Output_Buff-X_Reg(:,s)));
    U_Input_Buff(1,1) =U_Input_Buff(1,1);
    U_Input_Buff(2,1) =U_Input_Buff(2,1);
    U_Input_Buff(3,1) =U_Input_Buff(3,1);
    U_Input_Buff(4,1) =U_Input_Buff(4,1);
    
    if (U_Input_Buff(1,1) <=0)
        U_Input_Buff(1,1) =0;
    end
    if (U_Input_Buff(2,1) <=0)
        U_Input_Buff(2,1) =0;
    end
    if (U_Input_Buff(3,1) <=0)
        U_Input_Buff(3,1) =0;
    end
    if (U_Input_Buff(4,1) <=0)
        U_Input_Buff(4,1) =0;
    end
    if (U_Input_Buff(1,1) >=Motor_Max)
        U_Input_Buff(1,1) =Motor_Max;
    end
    if (U_Input_Buff(2,1) >=Motor_Max)
        U_Input_Buff(2,1) =Motor_Max;
    end
    if (U_Input_Buff(3,1) >=Motor_Max)
        U_Input_Buff(3,1) =Motor_Max;
    end
    if (U_Input_Buff(4,1) >=Motor_Max)
        U_Input_Buff(4,1) =Motor_Max;
    end
    
    %Calculation motor speed and maximum speed
    F_input_buff(1,1) = U_Input_Buff(1,1);
    F_input_buff(2,1) = U_Input_Buff(2,1);
    F_input_buff(3,1) = U_Input_Buff(3,1);
    F_input_buff(4,1) = U_Input_Buff(4,1);

    %Saving results to matrix
    Y_Output(1,s) = Y_Output_Buff(4,1);
    Y_Output(2,s) = Y_Output_Buff(5,1);
    Y_Output(3,s) = Y_Output_Buff(6,1);
    
    Y_Output_Noise(1,s) = Y_Output_Noise_Buff(1,1);
    Y_Output_Noise(2,s) = Y_Output_Noise_Buff(5,1);
    Y_Output_Noise(3,s) = Y_Output_Noise_Buff(6,1);
    
    Y_Output_Filtered(1,s) = Y_Output_Filtered_Buff(1,1);
    Y_Output_Filtered(2,s) = Y_Output_Filtered_Buff(5,1);
    Y_Output_Filtered(3,s) = Y_Output_Filtered_Buff(6,1);

    
    U_Input(1,s) = U_Input_Buff(1,1);
    U_Input(2,s) = U_Input_Buff(2,1);
    U_Input(3,s) = U_Input_Buff(3,1);
    U_Input(4,s) = U_Input_Buff(4,1);
    
    F_Input(1,s) = F_input_buff(1,1);
    F_Input(2,s) = F_input_buff(2,1);
    F_Input(3,s) = F_input_buff(3,1);
    F_Input(4,s) = F_input_buff(4,1);
end

%Plot

T = [Tp:Tp:T_End];
Plot_1 = figure('Position', [10 10 650 400]);
subplot(2,1,1);
plot(T,Y_Output(1,:),'b')
grid;
title('LQR regulator - Original Y output');
legend('Z')
ylabel('Position [m]');
xlabel('Time [s]');
hold off

subplot(2,1,2);
grid;
plot(T,Y_Output(2,:),'b')
hold on
plot(T,Y_Output(3,:),'g')
grid;
legend('FI','THETA')
ylabel('Degree ["]');
xlabel('TIme [s]');
saveas(Plot_1,'MD_LQR_Original_Y.png')

Plot_2 = figure('Position', [10 10 650 400]);
subplot(2,1,1);
plot(T,Y_Output_Noise(1,:),'b')
grid;
hold on;
plot(T,Y_Output_Filtered(1,:),'g')
grid;
title('LQR regulator - Noise and filtered Y output');
legend('AZ_N','AZ_F')
ylabel('Accel [m/s*2]');
xlabel('Time [s]');
hold off

subplot(2,1,2);
plot(T,Y_Output_Noise(2,:),'b')
grid;
hold on
plot(T,Y_Output_Noise(3,:),'c')
grid;
hold on;
plot(T,Y_Output_Filtered(2,:),'b')
grid;
hold on
plot(T,Y_Output_Filtered(3,:),'c')
grid;

legend('FI_N','THETA_N', 'FI_F','THETA_F')
ylabel('Degree ["]');
xlabel('Time [s]');
saveas(Plot_2,'MD_LQR_Filtered_Y.png')

Plot_3 = figure('Position', [10 10 650 400]);
plot(T,F_Input(1,:),'b')
grid;
hold on
plot(T,F_Input(2,:),'c')
grid;
hold on
plot(T,F_Input(3,:),'g')
grid;
hold on
plot(T,F_Input(4,:),'r')
grid;
title('LQR regulator - Required forces');
legend('F1','F2','F3', 'F4')
ylabel('Force [N]');
xlabel('Time [s]');
saveas(Plot_3,'MD_LQR_Forces.png')
Done = 1;
end