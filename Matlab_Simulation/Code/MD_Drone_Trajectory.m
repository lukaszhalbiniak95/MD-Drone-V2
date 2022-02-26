% MD Drone - Simulation code for 4 motor micro drone
% Task: Trajectory generator
% Author: £ukasz Halbiniak

function X_Reg = MD_Drone_Trajectory(Sample)

X_Reg = zeros(6,Sample);
for c = 1:Sample
    if c <= (Sample/8)*1
        X_Reg(4,c)= 1;
        X_Reg(5,c)= 20;
        X_Reg(6,c)= 30;

    end
    if c <= (Sample/8)*2 && c > (Sample/8)*1
        X_Reg(4,c)= 2;
        X_Reg(5,c)= 20;
        X_Reg(6,c)= 0;
    end
    if c <= (Sample/8)*3 && c > (Sample/8)*2
        X_Reg(4,c)= 1;
        X_Reg(5,c)= 0;
        X_Reg(6,c)= 20;
    end
    if c <= (Sample/8)*4 && c > (Sample/8)*3
        X_Reg(4,c)= 1;
        X_Reg(5,c)= 0;
        X_Reg(6,c)= 0;

    end
    if c <= (Sample/8)*5 && c > (Sample/8)*4
        X_Reg(4,c)= 1;
        X_Reg(5,c)= 20;
        X_Reg(6,c)= 20;
    end
    if c <= (Sample/8)*6 && c > (Sample/8)*5
        X_Reg(4,c)= 1;
        X_Reg(5,c)= 0;
        X_Reg(6,c)= 0;
    end
    if c <= (Sample/8)*7 && c > (Sample/8)*8
        X_Reg(4,c)= 0;
        X_Reg(5,c)= 0;
        X_Reg(6,c)= 0;
    end
end

end