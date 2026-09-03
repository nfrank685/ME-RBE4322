%six bar linkage
%static equilibrium

clc;
clear;

%define the joints
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
F = [43 32 0];
G = [45 17 0];

%joint values

new_B_x(1) = B(1);
new_B_y(1) = B(2);
new_C_x(1) = C(1);
new_C_y(1) = C(2);
new_D_x(1) = D(1);
new_D_y(1) = D(2);
new_E_x(1) = E(1);
new_E_y(1) = E(2);
new_F_x(1) = F(1);
new_F_y(1) = F(2);


% Define the lengths of the bars connecting the joints
AB = norm(B - A);
BC = norm(C - B);
CD = norm(D - C);
BE = norm(E - B);
EF = norm(F - E);
FG = norm(G - F);
CE = norm(E - C);

%Weight of each link
WAB = [0 -1 0];
WBEC = [0 -1 0];
WCD = [0 -1 0];
WEF = [0 -1 0];
WFG = [0 -1 0];

%center of mass of each link
S1 = (A+B)/2;
S2 = (B+C+E)/3;
S3 = (C+D)/2;
S4 = (E+F)/2;
S5 = (F+G)/2;

syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

ForceA = [FAx FAy 0];
ForceB = [FBx FBy 0];
ForceC = [FCx FCy 0];
ForceD = [FDx FDy 0];
ForceE = [FEx FEy 0];
ForceF = [FFx FFy 0];
ForceG = [FGx FGy 0];
InputTorque = [0 0 Tin];

%Applied Force
AppliedForce = [50 0 0];

%Static equilibirum conditions for link AB

%Sum of forces = 0
%Fa + Fb + WeightofAB = 0
eqn1 = ForceA + ForceB + WAB == 0;
%Sum of moments = 0 w/ respect to CoM of link AB
%S1A X FA + S1B X FB + InputTorque = 0
eqn2 = cross(A-S1,ForceA) + cross(B-S1,ForceB) + InputTorque == 0;


%Equations for Link BEC
%%Sum of forces = 0
% -Fb + Fc + Fe + WBEC = 0

eqn3 = -ForceB + ForceC + ForceE + WBEC ==0;

%Sum of moments = 0
%Sum of moments = 0 w/ respect to CoM of Link BEC
% S2B X -FB + S2C X FC + S2E X FE = 0
eqn4 = cross(B-S2, -ForceB) + cross(C-S2, ForceC) + cross(E-S2, ForceE) == 0;

%Equations for Link CD
%Sum of forces = 0
% -Fc + Fd + WCD = 0
eqn5 = -ForceC + ForceD + WCD == 0;

%Sum of moments = 0
%Sum of moments = 0 w/ respect to CoM of link CD
% S3C X -FC + S3D X FD = 0
eqn6 = cross(C-S3, -ForceC) + cross(D-S3, ForceD) == 0;

%Equations for link EF
%Sum of forces = 0
eqn7 = -ForceE + ForceF + WEF ==0;

%Sum of moments = 0
%S4E X -FE +S4E X FF = 0
eqn8 = cross(E-S4,-ForceE) + cross(F-S4,ForceF) ==0;

%Equations for Link FG
%Sum of forces = 0
eqn9 = -ForceF + ForceG + WFG + AppliedForce== 0;

%Sum of moments = 0
eqn10 = cross(F-S5, -ForceF) + cross(G-S5, ForceG) ==0;

%Solve the 10 equations

eqnMatrix = [eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8, eqn9, eqn10];

staticSolution = solve(eqnMatrix, [FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin]);

Force_Ax = double(staticSolution.FAx);
Force_Ay = double(staticSolution.FAy);
Force_Bx = double(staticSolution.FBx);
Force_By = double(staticSolution.FBy);
Force_Cx = double(staticSolution.FCx);
Force_Cy = double(staticSolution.FCy);
Force_Dx = double(staticSolution.FDx);
Force_Dy = double(staticSolution.FDy);
Force_Ex = double(staticSolution.FEx);
Force_Ey = double(staticSolution.FEy);
Force_Fx = double(staticSolution.FFx);
Force_Fy = double(staticSolution.FFy);
Force_Gx = double(staticSolution.FGx);
Force_Gy = double(staticSolution.FGy);
Torque_in = double(staticSolution.Tin);

disp("Force A: ");
disp([Force_Ax, Force_Ay]);
disp("Force B: ");
disp([Force_Bx, Force_By]);
disp("Force C: ");
disp([Force_Cx, Force_Cy]);
disp("Force D: ");
disp([Force_Dx, Force_Dy]);
disp("Force E: ");
disp([Force_Ex, Force_Ey]);
disp("Force F: ");
disp([Force_Fx, Force_Fy]);
disp("Force G: ");
disp([Force_Gx, Force_Gy]);
disp("Input Torque: ");
disp(InputTorque);

%angular velocity calculations

%Loop ABCDA

syms wBEC wCD
omega_AB = [0 0 1];
omega_BEC = [0 0 wBEC];
omega_CD = [0 0 wCD];

eqn11 = cross(omega_AB,B-A) + cross(omega_BEC,C-B) + cross(omega_CD,D-C) == 0;
loop1Solution = solve(eqn11,[wBEC wCD]);

%extract angular velocities from the loop solution
angularVeloctiy_BEC = double(loop1Solution.wBEC)
angularVeloctiy_CD = double(loop1Solution.wCD)


%Loop DEFGD
omega_BEC = [0 0 angularVeloctiy_BEC];
omega_CD = [0 0 angularVeloctiy_CD];

syms wEF wFG
omega_EF = [0 0 wEF];
omega_FG = [0 0 wFG];


eqn12 = cross(omega_CD,C-D) + cross(omega_BEC,E-C) + cross(omega_EF,F-E) + cross(omega_FG, G-F) == 0;

loop2Solution = solve (eqn12,[wEF,wFG]);

angularVeloctiy_EF = double(loop2Solution.wEF)
angularVeloctiy_FG = double(loop2Solution.wFG)

%angular Acceleration for loop 1 ABCDA
syms aBEC aCD
alpha_AB = [0 0 0];
alpha_BEC = [0 0 aBEC];
alpha_CD = [0 0 aCD];

a_B_A = cross(alpha_AB,B-A) + cross(omega_AB,cross(omega_AB,B-A));
a_C_B = cross(alpha_BEC,C-B) + cross(omega_BEC,cross(omega_BEC,C-B));
a_D_C = cross(alpha_CD,D-C) + cross(omega_CD,cross(omega_CD,D-C));

eqn13 = a_B_A + a_C_B + a_D_C == 0;

loop1AccSolution = solve(eqn13,[aBEC aCD]);
alphaBEC = double(loop1AccSolution.aBEC);
alphaCD = double(loop1AccSolution.aCD);

alphaBEC_vector = [0 0 alphaBEC];
alphaCD_vector = [0 0 alphaCD];

% angular acceleration for loop 2 DEFGD
syms aEF aFG

alpha_EF = [0 0 aEF];
alpha_FG = [0 0 aFG];

a_C_D = cross(alphaCD_vector,C-D) + ...
        cross(omega_CD,cross(omega_CD,C-D));

a_E_C = cross(alphaBEC_vector,E-C) + ...
        cross(omega_BEC,cross(omega_BEC,E-C));

% Use the numerical angular velocities found above
angVel_EF = [0 0 angularVeloctiy_EF];
angVel_FG = [0 0 angularVeloctiy_FG];

a_E_F = cross(alpha_EF,F-E) + ...
        cross(angVel_EF,cross(angVel_EF,F-E));

a_F_G = cross(alpha_FG,G-F) + ...
        cross(angVel_FG,cross(angVel_FG,G-F));

eqn14 = a_C_D + a_E_C + a_E_F + a_F_G == 0;

loop2AccSolution = solve(eqn14,[aEF aFG]);

% Extract angular accelerations
alphaEF = double(loop2AccSolution.aEF);
alphaFG = double(loop2AccSolution.aFG);
%velocity at a joint

vB_A = cross(omega_AB,B-A);

vE_B = cross(omega_BEC,E-B);

vF_G = cross(angVel_FG,F-G);

vC_D = cross(omega_CD, C-D);

vC_B = cross(omega_BEC,C-B);

vF_E = cross(angVel_EF,F-E);

vC_A = vC_B + vB_A;

vE_A = vE_B + vB_A;

vF_A = vF_E + vE_A;

vS1_A = cross(omega_AB,S1-A);

vS2_B = cross(omega_BEC,S2-B);

vS3_D = cross(omega_CD,S3-D);

vS5_G = cross(angVel_FG,S5-G);

vS1_A = cross(omega_AB,S1-A);

vS2_A = cross(omega_BEC,S2-B) + vB_A;

vS3_A = cross(omega_CD,S3-D);

vS4_A = cross(angVel_EF,S4-E) + vE_A;

vS5_A = cross(angVel_FG,S5-G);

aF_G = cross([0 0 alphaFG], F-G) + cross(angVel_FG, cross(angVel_FG, F-G));

%accelerations at COM every link
aS1_A = cross(alpha_AB, S1-A) + cross(omega_AB, cross(omega_AB, S1-A));

aS2_A = cross(alphaBEC_vector,S2-B) + cross(omega_BEC,cross(omega_BEC,S2-B)) + a_B_A;

aS3_D = cross(alphaCD_vector, S3-D) + cross(omega_CD, cross(omega_CD,S3-D));

aS4_G = cross([0 0 alphaEF], S4-F) + cross(angVel_EF, cross(angVel_EF,S4-F)) + aF_G;

aS5_G = cross([0 0 alphaFG], S5-G) + cross(angVel_FG, cross(angVel_FG, S5-G));

%Newtons second law

MassAB = 1;
MassBEC = 1;
MassCD = 1;
MassEF = 1;
MassFG = 1;

%Mass moment of inertia
J_AB = 1;
J_BEC = 1;
J_CD = 1;
J_EF = 1;
J_FG = 1;

syms NFAx  NFAy NFBx NFBy NFCx NFCy NFDx NFDy NFEx NFEy NFFx NFFy NFGx NFGy NTin

%Define forces

NForceA = [NFAx NFAy 0];
NForceB = [NFBx NFBy 0];
NForceC = [NFCx NFCy 0];
NForceD = [NFDx NFDy 0];
NForceE = [NFEx NFEy 0];
NForceF = [NFFx NFFy 0];
NForceG = [NFGx NFGy 0];
NInputTorque = [0 0 NTin];

%equations for link AB
%Sum of forces
eqn15 = NForceA + NForceB + WAB == MassAB * aS1_A;
%Sum of moments
eqn16 = cross(A-S1, NForceA) + cross(B-S1,NForceB) + NInputTorque == J_AB * alpha_AB;

%equations for Link BEC
%Sum of forces
eqn17 = -NForceB + NForceC + NForceE + WBEC == MassBEC * aS2_A;

%Sum of moments
eqn18 = cross(B-S2, -NForceB) + cross(C-S2,NForceC) + cross(E-S2, NForceE) == J_BEC * alphaBEC_vector;

%equations for link CD
%sum of forces
eqn19 = -NForceC + NForceD + WCD == MassCD * aS3_D;

%Sum of moments
eqn20 = cross(C-S3, -NForceC) + cross(D-S3,NForceD) == J_CD * alphaCD_vector;

%equations for link EF
%sum of forces
eqn21 = -NForceE + NForceF + WEF == MassEF * aS4_G;

%Sum of moments
eqn22 = cross(E-S4,-NForceE) + cross(F-S4,NForceF) == J_EF * alphaEF;

%equations for link FG
%sum of forces
eqn23 = -NForceF + NForceG + WFG + AppliedForce == MassFG * aS5_G;

%Sum of moments
eqn24 = cross(F-S5,-NForceF) + cross(G-S5, NForceG) == J_FG * [0 0 alphaFG];

%solving equations
NeqMatrix = [eqn15, eqn16, eqn17, eqn18, eqn19, eqn20, eqn21, eqn22, eqn23, eqn24];
DynamicSolution = solve(NeqMatrix, [NFAx NFAy NFBx NFBy NFCx NFCy NFDx NFDy NFEx NFEy NFFx NFFy NFGx NFGy NTin]);



% Extract forces from the dynamic solution
NForce_Ax = double(DynamicSolution.NFAx);
NForce_Ay = double(DynamicSolution.NFAy);
NForce_Bx = double(DynamicSolution.NFBx);
NForce_By = double(DynamicSolution.NFBy);
NForce_Cx = double(DynamicSolution.NFCx);
NForce_Cy = double(DynamicSolution.NFCy);
NForce_Dx = double(DynamicSolution.NFDx);
NForce_Dy = double(DynamicSolution.NFDy);
NForce_Ex = double(DynamicSolution.NFEx);
NForce_Ey = double(DynamicSolution.NFEy);
NForce_Fx = double(DynamicSolution.NFFx);
NForce_Fy = double(DynamicSolution.NFFy);
NForce_Gx = double(DynamicSolution.NFGx);
NForce_Gy = double(DynamicSolution.NFGy);
Torque_out = double(DynamicSolution.NTin);

% Extract forces from the dynamic solution
disp("Dynamic Forces:");
disp([NForce_Ax, NForce_Ay]);
disp([NForce_Bx, NForce_By]);
disp([NForce_Cx, NForce_Cy]);
disp([NForce_Dx, NForce_Dy]);
disp([NForce_Ex, NForce_Ey]);
disp([NForce_Fx, NForce_Fy]);
disp([NForce_Gx, NForce_Gy]);
disp("Output Torque: ");
disp(Torque_out);

%Circle intersection technique

%joint coordinates have been defined
%length of links also defined

%compute initial angle of input link AB

initial_theta = atan2(B(2)-A(2),B(1)-A(1));

if (initial_theta < 0)
    inputAngle = 2*pi + initial_theta;
else 
    inputAngle = initial_theta;
end

for (theta = 1:1:360)
    %new position of joint B
    B_new = A + [AB*cos(inputAngle + deg2rad(theta)) AB*sin(inputAngle + deg2rad(theta)) 0];
    %new position of C
    %w/ B_new as center, BC as radius
    %w/ D as center and DC as radius
    [Cx, Cy] = circcirc(B_new(1), B_new(2), BC, D(1),D(2), CD);

    %checking if there is a NaN

    circIntersect_x = any(isnan(vpa(Cx)));
    circIntersect_y = any(isnan(vpa(Cy)));
    
    if circIntersect_x == 0 && circIntersect_y ==0
        C_1 = [Cx(1) Cy(1) 0];
        C_2 = [Cx(2) Cy(2) 0];

        dist1 = norm(C_1 - C);
        dist2 = norm(C_2 - C);

        if(dist1 < dist2)
            C_new = vpa(C_1); 
            C = C_new; 
        else 
            C_new = vpa(C_2);
            C = C_new;
        end

        
        %New Position of joint E using B_new and C_new
        [Ex, Ey] = circcirc(B_new(1), B_new(2), BE, C_new(1),C_new(2), CE);
        circIntersect_x = any(isnan(vpa(Ex)));
        circIntersect_y = any(isnan(vpa(Ey)));

        if circIntersect_x == 0 && circIntersect_y ==0
            E_1 = [Ex(1) Ey(1) 0];
            E_2 = [Ex(2) Ey(2) 0];

            dist1 = norm(E_1 - E);
            dist2 = norm(E_2 - E);

            if(dist1 < dist2)
                E_new = vpa(E_1);
            else 
                E_new = vpa(E_2);
            end

        else 
            fprintf("New position of E cannot be determined at angle %d degree", theta);
        end
    
        %New Position of joint E using B_new and C_new
        [Fx, Fy] = circcirc(E_new(1), E_new(2), EF, G(1), G(2), FG);
        circIntersect_x = any(isnan(vpa(Fx)));
        circIntersect_y = any(isnan(vpa(Fy)));

        if circIntersect_x == 0 && circIntersect_y ==0
            F_1 = [Fx(1) Fy(1) 0];
            F_2 = [Fx(2) Fy(2) 0];

            dist1 = norm(F_1 - F);
            dist2 = norm(F_2 - F);

            if(dist1 < dist2)
                F_new = vpa(F_1);
            else 
                F_new = vpa(F_2);
            end

           %store values for plotting 
            new_B_x(theta+1) = B_new(1);
            new_B_y(theta+1) = B_new(2);
            new_C_x(theta+1) = C_new(1);
            new_C_y(theta+1) = C_new(2);
            new_D_x(theta+1) = D(1);
            new_D_y(theta+1) = D(2);
            new_E_x(theta+1) = E_new(1);
            new_E_y(theta+1) = E_new(2);
            new_F_x(theta+1) = F_new(1);
            new_F_y(theta+1) = F_new(2);

            B = B_new;
            C = C_new;
            E = E_new;
            F = F_new;

            %Static equilibirum conditions for link AB

% Define the lengths of the bars connecting the joints
AB = norm(B - A);
BC = norm(C - B);
CD = norm(D - C);
BE = norm(E - B);
EF = norm(F - E);
FG = norm(G - F);
CE = norm(E - C);

%Weight of each link
WAB = [0 -1 0];
WBEC = [0 -1 0];
WCD = [0 -1 0];
WEF = [0 -1 0];
WFG = [0 -1 0];

%center of mass of each link
S1 = (A+B)/2;
S2 = (B+C+E)/3;
S3 = (C+D)/2;
S4 = (E+F)/2;
S5 = (F+G)/2;

syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

ForceA = [FAx FAy 0];
ForceB = [FBx FBy 0];
ForceC = [FCx FCy 0];
ForceD = [FDx FDy 0];
ForceE = [FEx FEy 0];
ForceF = [FFx FFy 0];
ForceG = [FGx FGy 0];
InputTorque = [0 0 Tin];

%Applied Force
AppliedForce = [50 0 0];

%Static equilibirum conditions for link AB

%Sum of forces = 0
%Fa + Fb + WeightofAB = 0
eqn1 = ForceA + ForceB + WAB == 0;
%Sum of moments = 0 w/ respect to CoM of link AB
%S1A X FA + S1B X FB + InputTorque = 0
eqn2 = cross(A-S1,ForceA) + cross(B-S1,ForceB) + InputTorque == 0;


%Equations for Link BEC
%%Sum of forces = 0
% -Fb + Fc + Fe + WBEC = 0

eqn3 = -ForceB + ForceC + ForceE + WBEC ==0;

%Sum of moments = 0
%Sum of moments = 0 w/ respect to CoM of Link BEC
% S2B X -FB + S2C X FC + S2E X FE = 0
eqn4 = cross(B-S2, -ForceB) + cross(C-S2, ForceC) + cross(E-S2, ForceE) == 0;

%Equations for Link CD
%Sum of forces = 0
% -Fc + Fd + WCD = 0
eqn5 = -ForceC + ForceD + WCD == 0;

%Sum of moments = 0
%Sum of moments = 0 w/ respect to CoM of link CD
% S3C X -FC + S3D X FD = 0
eqn6 = cross(C-S3, -ForceC) + cross(D-S3, ForceD) == 0;

%Equations for link EF
%Sum of forces = 0
eqn7 = -ForceE + ForceF + WEF ==0;

%Sum of moments = 0
%S4E X -FE +S4E X FF = 0
eqn8 = cross(E-S4,-ForceE) + cross(F-S4,ForceF) ==0;

%Equations for Link FG
%Sum of forces = 0
eqn9 = -ForceF + ForceG + WFG + AppliedForce== 0;

%Sum of moments = 0
eqn10 = cross(F-S5, -ForceF) + cross(G-S5, ForceG) ==0;

%Solve the 10 equations

eqnMatrix = [eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8, eqn9, eqn10];

staticSolution = solve(eqnMatrix, [FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin]);

Force_Ax = double(staticSolution.FAx);
Force_Ay = double(staticSolution.FAy);
Force_Bx = double(staticSolution.FBx);
Force_By = double(staticSolution.FBy);
Force_Cx = double(staticSolution.FCx);
Force_Cy = double(staticSolution.FCy);
Force_Dx = double(staticSolution.FDx);
Force_Dy = double(staticSolution.FDy);
Force_Ex = double(staticSolution.FEx);
Force_Ey = double(staticSolution.FEy);
Force_Fx = double(staticSolution.FFx);
Force_Fy = double(staticSolution.FFy);
Force_Gx = double(staticSolution.FGx);
Force_Gy = double(staticSolution.FGy);
Torque_in = double(staticSolution.Tin);

disp("Force A: ");
disp([Force_Ax, Force_Ay]);
disp("Force B: ");
disp([Force_Bx, Force_By]);
disp("Force C: ");
disp([Force_Cx, Force_Cy]);
disp("Force D: ");
disp([Force_Dx, Force_Dy]);
disp("Force E: ");
disp([Force_Ex, Force_Ey]);
disp("Force F: ");
disp([Force_Fx, Force_Fy]);
disp("Force G: ");
disp([Force_Gx, Force_Gy]);
disp("Input Torque: ");
disp(InputTorque);

%angular velocity calculations

%Loop ABCDA

syms wBEC wCD
omega_AB = [0 0 1];
omega_BEC = [0 0 wBEC];
omega_CD = [0 0 wCD];

eqn11 = cross(omega_AB,B-A) + cross(omega_BEC,C-B) + cross(omega_CD,D-C) == 0;
loop1Solution = solve(eqn11,[wBEC wCD]);

%extract angular velocities from the loop solution
angularVeloctiy_BEC = double(loop1Solution.wBEC)
angularVeloctiy_CD = double(loop1Solution.wCD)


%Loop DEFGD
omega_BEC = [0 0 angularVeloctiy_BEC];
omega_CD = [0 0 angularVeloctiy_CD];

syms wEF wFG
omega_EF = [0 0 wEF];
omega_FG = [0 0 wFG];


eqn12 = cross(omega_CD,C-D) + cross(omega_BEC,E-C) + cross(omega_EF,F-E) + cross(omega_FG, G-F) == 0;

loop2Solution = solve (eqn12,[wEF,wFG]);

angularVeloctiy_EF = double(loop2Solution.wEF)
angularVeloctiy_FG = double(loop2Solution.wFG)

%angular Acceleration for loop 1 ABCDA
syms aBEC aCD
alpha_AB = [0 0 0];
alpha_BEC = [0 0 aBEC];
alpha_CD = [0 0 aCD];

a_B_A = cross(alpha_AB,B-A) + cross(omega_AB,cross(omega_AB,B-A));
a_C_B = cross(alpha_BEC,C-B) + cross(omega_BEC,cross(omega_BEC,C-B));
a_D_C = cross(alpha_CD,D-C) + cross(omega_CD,cross(omega_CD,D-C));

eqn13 = a_B_A + a_C_B + a_D_C == 0;

loop1AccSolution = solve(eqn13,[aBEC aCD]);
alphaBEC = double(loop1AccSolution.aBEC);
alphaCD = double(loop1AccSolution.aCD);

alphaBEC_vector = [0 0 alphaBEC];
alphaCD_vector = [0 0 alphaCD];

% angular acceleration for loop 2 DEFGD
syms aEF aFG

alpha_EF = [0 0 aEF];
alpha_FG = [0 0 aFG];

a_C_D = cross(alphaCD_vector,C-D) + ...
        cross(omega_CD,cross(omega_CD,C-D));

a_E_C = cross(alphaBEC_vector,E-C) + ...
        cross(omega_BEC,cross(omega_BEC,E-C));

% Use the numerical angular velocities found above
angVel_EF = [0 0 angularVeloctiy_EF];
angVel_FG = [0 0 angularVeloctiy_FG];

a_E_F = cross(alpha_EF,F-E) + ...
        cross(angVel_EF,cross(angVel_EF,F-E));

a_F_G = cross(alpha_FG,G-F) + ...
        cross(angVel_FG,cross(angVel_FG,G-F));

eqn14 = a_C_D + a_E_C + a_E_F + a_F_G == 0;

loop2AccSolution = solve(eqn14,[aEF aFG]);

% Extract angular accelerations
alphaEF = double(loop2AccSolution.aEF);
alphaFG = double(loop2AccSolution.aFG);
%velocity at a joint

vB_A = cross(omega_AB,B-A);

vE_B = cross(omega_BEC,E-B);

vF_G = cross(angVel_FG,F-G);

vC_D = cross(omega_CD, C-D);

vC_B = cross(omega_BEC,C-B);

vF_E = cross(angVel_EF,F-E);

vC_A = vC_B + vB_A;

vE_A = vE_B + vB_A;

vF_A = vF_E + vE_A;

vS1_A = cross(omega_AB,S1-A);

vS2_B = cross(omega_BEC,S2-B);

vS3_D = cross(omega_CD,S3-D);

vS5_G = cross(angVel_FG,S5-G);

vS1_A = cross(omega_AB,S1-A);

vS2_A = cross(omega_BEC,S2-B) + vB_A;

vS3_A = cross(omega_CD,S3-D);

vS4_A = cross(angVel_EF,S4-E) + vE_A;

vS5_A = cross(angVel_FG,S5-G);

aF_G = cross([0 0 alphaFG], F-G) + cross(angVel_FG, cross(angVel_FG, F-G));

%accelerations at COM every link
aS1_A = cross(alpha_AB, S1-A) + cross(omega_AB, cross(omega_AB, S1-A));

aS2_A = cross(alphaBEC_vector,S2-B) + cross(omega_BEC,cross(omega_BEC,S2-B)) + a_B_A;

aS3_D = cross(alphaCD_vector, S3-D) + cross(omega_CD, cross(omega_CD,S3-D));

aS4_G = cross([0 0 alphaEF], S4-F) + cross(angVel_EF, cross(angVel_EF,S4-F)) + aF_G;

aS5_G = cross([0 0 alphaFG], S5-G) + cross(angVel_FG, cross(angVel_FG, S5-G));

%Newtons second law

MassAB = 1;
MassBEC = 1;
MassCD = 1;
MassEF = 1;
MassFG = 1;

%Mass moment of inertia
J_AB = 1;
J_BEC = 1;
J_CD = 1;
J_EF = 1;
J_FG = 1;

syms NFAx  NFAy NFBx NFBy NFCx NFCy NFDx NFDy NFEx NFEy NFFx NFFy NFGx NFGy NTin

%Define forces

NForceA = [NFAx NFAy 0];
NForceB = [NFBx NFBy 0];
NForceC = [NFCx NFCy 0];
NForceD = [NFDx NFDy 0];
NForceE = [NFEx NFEy 0];
NForceF = [NFFx NFFy 0];
NForceG = [NFGx NFGy 0];
NInputTorque = [0 0 NTin];

%equations for link AB
%Sum of forces
eqn15 = NForceA + NForceB + WAB == MassAB * aS1_A;
%Sum of moments
eqn16 = cross(A-S1, NForceA) + cross(B-S1,NForceB) + NInputTorque == J_AB * alpha_AB;

%equations for Link BEC
%Sum of forces
eqn17 = -NForceB + NForceC + NForceE + WBEC == MassBEC * aS2_A;

%Sum of moments
eqn18 = cross(B-S2, -NForceB) + cross(C-S2,NForceC) + cross(E-S2, NForceE) == J_BEC * alphaBEC_vector;

%equations for link CD
%sum of forces
eqn19 = -NForceC + NForceD + WCD == MassCD * aS3_D;

%Sum of moments
eqn20 = cross(C-S3, -NForceC) + cross(D-S3,NForceD) == J_CD * alphaCD_vector;

%equations for link EF
%sum of forces
eqn21 = -NForceE + NForceF + WEF == MassEF * aS4_G;

%Sum of moments
eqn22 = cross(E-S4,-NForceE) + cross(F-S4,NForceF) == J_EF * alphaEF;

%equations for link FG
%sum of forces
eqn23 = -NForceF + NForceG + WFG + AppliedForce == MassFG * aS5_G;

%Sum of moments
eqn24 = cross(F-S5,-NForceF) + cross(G-S5, NForceG) == J_FG * [0 0 alphaFG];

%solving equations
NeqMatrix = [eqn15, eqn16, eqn17, eqn18, eqn19, eqn20, eqn21, eqn22, eqn23, eqn24];
DynamicSolution = solve(NeqMatrix, [NFAx NFAy NFBx NFBy NFCx NFCy NFDx NFDy NFEx NFEy NFFx NFFy NFGx NFGy NTin]);



% Extract forces from the dynamic solution
NForce_Ax = double(DynamicSolution.NFAx);
NForce_Ay = double(DynamicSolution.NFAy);
NForce_Bx = double(DynamicSolution.NFBx);
NForce_By = double(DynamicSolution.NFBy);
NForce_Cx = double(DynamicSolution.NFCx);
NForce_Cy = double(DynamicSolution.NFCy);
NForce_Dx = double(DynamicSolution.NFDx);
NForce_Dy = double(DynamicSolution.NFDy);
NForce_Ex = double(DynamicSolution.NFEx);
NForce_Ey = double(DynamicSolution.NFEy);
NForce_Fx = double(DynamicSolution.NFFx);
NForce_Fy = double(DynamicSolution.NFFy);
NForce_Gx = double(DynamicSolution.NFGx);
NForce_Gy = double(DynamicSolution.NFGy);
Torque_out = double(DynamicSolution.NTin);

% Extract forces from the dynamic solution
disp("Dynamic Forces:");
disp([NForce_Ax, NForce_Ay]);
disp([NForce_Bx, NForce_By]);
disp([NForce_Cx, NForce_Cy]);
disp([NForce_Dx, NForce_Dy]);
disp([NForce_Ex, NForce_Ey]);
disp([NForce_Fx, NForce_Fy]);
disp([NForce_Gx, NForce_Gy]);
disp("Output Torque: ");
disp(Torque_out);

        else 
            fprintf("New position of F cannot be determined at angle %d degree", theta);
        end


       
    else 
        fprintf("New position of C cannot be determined at angle %d degree", theta);
    end

   


end 

