
%ME/RBE 4322 HW#1 - Six-bar linkage


clc;
clear;
close all;


%Joint coordinates
A = [ 1.4, 0.485, 0 ];
B = [ 1.67, 0.99, 0 ];
C = [ 0.255, 1.035, 0 ];
D = [ 0.285, 0.055, 0 ];
E = [ 0.195, 2.54, 0 ];
F = [-0.98, 2.57, 0 ];
G = [ 0.05, 0.2, 0 ];

% Link lengths 
AB = 0.5726;       
BC = 1.4157;      
DE = 2.4866;      
EF = 1.1754;      
GF = 2.5841;       

% Useful distances for the initial configuration
CD0 = norm(C-D);
CE0 = norm(E-C);
BE0 = norm(E-B);

% C is a fixed point on rigid link DE.
% Fraction of the D-to-E distance at which joint C is located:
C_fraction = CD0/DE;

% Check the supplied geometry against the supplied link lengths.

fprintf('  AB: %.6f m (PDF %.6f m)\n', norm(B-A), AB);
fprintf('  BC: %.6f m (PDF %.6f m)\n', norm(C-B), BC);
fprintf('  DE: %.6f m (PDF %.6f m)\n', norm(E-D), DE);
fprintf('  EF: %.6f m (PDF %.6f m)\n', norm(F-E), EF);
fprintf('  GF: %.6f m (PDF %.6f m)\n\n', norm(G-F), GF);


rho = 7850;          
g = 9.81;           

% Cross-section shown in the PDF
linkWidth = 0.10;    % m
linkThickness = 0.05;% m
jointHoleD = 0.06;   % m

% Mass and mass moment of inertia for each link
% The link is modeled as a rectangular middle section with two
% semicircular ends and two circular joint holes.
r = linkWidth/2;
rh = jointHoleD/2;

% Area of the rectangular middle section and the two semicircular ends
Arect1 = AB*linkWidth; Acircle1 = pi*r^2; Ahole1 = pi*rh^2;
Arect2 = BC*linkWidth; Acircle2 = pi*r^2; Ahole2 = pi*rh^2;
Arect3 = DE*linkWidth; Acircle3 = pi*r^2; Ahole3 = pi*rh^2;
Arect4 = EF*linkWidth; Acircle4 = pi*r^2; Ahole4 = pi*rh^2;
Arect5 = GF*linkWidth; Acircle5 = pi*r^2; Ahole5 = pi*rh^2;

% Mass of each link
MassAB = rho*linkThickness*(Arect1 + Acircle1 - 2*Ahole1);
MassBC = rho*linkThickness*(Arect2 + Acircle2 - 2*Ahole2);
MassDE = rho*linkThickness*(Arect3 + Acircle3 - 2*Ahole3);
MassEF = rho*linkThickness*(Arect4 + Acircle4 - 2*Ahole4);
MassFG = rho*linkThickness*(Arect5 + Acircle5 - 2*Ahole5);

% Component masses used for the mass moments of inertia
mRect1 = rho*linkThickness*Arect1; mEnd1 = rho*linkThickness*Acircle1; mHole1 = rho*linkThickness*Ahole1;
mRect2 = rho*linkThickness*Arect2; mEnd2 = rho*linkThickness*Acircle2; mHole2 = rho*linkThickness*Ahole2;
mRect3 = rho*linkThickness*Arect3; mEnd3 = rho*linkThickness*Acircle3; mHole3 = rho*linkThickness*Ahole3;
mRect4 = rho*linkThickness*Arect4; mEnd4 = rho*linkThickness*Acircle4; mHole4 = rho*linkThickness*Ahole4;
mRect5 = rho*linkThickness*Arect5; mEnd5 = rho*linkThickness*Acircle5; mHole5 = rho*linkThickness*Ahole5;

% Mass moment of inertia about the centroidal z-axis
J_AB = mRect1*(AB^2 + linkWidth^2)/12 + mEnd1*(r^2/2 + (AB/2)^2) - 2*mHole1*(rh^2/2 + (AB/2)^2);
J_BC = mRect2*(BC^2 + linkWidth^2)/12 + mEnd2*(r^2/2 + (BC/2)^2) - 2*mHole2*(rh^2/2 + (BC/2)^2);
J_DE = mRect3*(DE^2 + linkWidth^2)/12 + mEnd3*(r^2/2 + (DE/2)^2) - 2*mHole3*(rh^2/2 + (DE/2)^2);
J_EF = mRect4*(EF^2 + linkWidth^2)/12 + mEnd4*(r^2/2 + (EF/2)^2) - 2*mHole4*(rh^2/2 + (EF/2)^2);
J_FG = mRect5*(GF^2 + linkWidth^2)/12 + mEnd5*(r^2/2 + (GF/2)^2) - 2*mHole5*(rh^2/2 + (GF/2)^2);

fprintf('Link properties:\n');
fprintf('  AB: mass = %.4f kg, J = %.4f kg-m^2\n', MassAB, J_AB);
fprintf('  BC: mass = %.4f kg, J = %.4f kg-m^2\n', MassBC, J_BC);
fprintf('  DE: mass = %.4f kg, J = %.4f kg-m^2\n', MassDE, J_DE);
fprintf('  EF: mass = %.4f kg, J = %.4f kg-m^2\n', MassEF, J_EF);
fprintf('  FG: mass = %.4f kg, J = %.4f kg-m^2\n\n', MassFG, J_FG);

% Weight vectors
WAB = [0, -MassAB*g, 0];
WBC = [0, -MassBC*g, 0];
WDE = [0, -MassDE*g, 0];
WEF = [0, -MassEF*g, 0];
WFG = [0, -MassFG*g, 0];


partsRequired = 12500;
productionTime = 9*3600;      
partsPerSecond = partsRequired/productionTime;

cycleTime = 1/partsPerSecond;
omegaInput = 2*pi/cycleTime;  


% Input angular velocity and acceleration
omega_AB = [0, 0, omegaInput];
alpha_AB = [0, 0, 0];        



artifactWeight = 200;      
gripperLength = 1.843;      


gripperDirection = (F-G)/norm(F-G);
P = F + gripperLength*gripperDirection;

AppliedForce = [0, -artifactWeight, 0];


%puts them into arrays for storage
nPos = 360;
thetaDeg = 0:nPos-1;
thetaRad = deg2rad(thetaDeg);

% Joint positions
B_pos = zeros(nPos,3);
C_pos = zeros(nPos,3);
D_pos = repmat(D,nPos,1);
E_pos = zeros(nPos,3);
F_pos = zeros(nPos,3);

% Joint velocities and accelerations
vB = zeros(nPos,3);
vC = zeros(nPos,3);
vE = zeros(nPos,3);
vF = zeros(nPos,3);

aB = zeros(nPos,3);
aC = zeros(nPos,3);
aE = zeros(nPos,3);
aF = zeros(nPos,3);

% Link angular velocity and angular acceleration (z components)
omegaBC = zeros(nPos,1);
omegaDE = zeros(nPos,1);
omegaEF = zeros(nPos,1);
omegaFG = zeros(nPos,1);

alphaBC = zeros(nPos,1);
alphaDE = zeros(nPos,1);
alphaEF = zeros(nPos,1);
alphaFG = zeros(nPos,1);

% Joint forces: columns are [x y]
staticForce = zeros(nPos,7,2);
dynamicForce = zeros(nPos,7,2);

% Joint order: A B C D E F G
staticTorque = zeros(nPos,1);
dynamicTorque = zeros(nPos,1);

% Mass-center locations, velocities, accelerations
S1_pos = zeros(nPos,3);
S2_pos = zeros(nPos,3);
S3_pos = zeros(nPos,3);
S4_pos = zeros(nPos,3);
S5_pos = zeros(nPos,3);

aS1 = zeros(nPos,3);
aS2 = zeros(nPos,3);
aS3 = zeros(nPos,3);
aS4 = zeros(nPos,3);
aS5 = zeros(nPos,3);



initialTheta = atan2(B(2)-A(2), B(1)-A(1));


B_prev = B;
C_prev = C;
E_prev = E;
F_prev = F;


syms SFAx SFAy SFBx SFBy SFCx SFCy SFDx SFDy SFEx SFEy SFFx SFFy SFGx SFGy STin

SForceA = [SFAx SFAy 0];
SForceB = [SFBx SFBy 0];
SForceC = [SFCx SFCy 0];
SForceD = [SFDx SFDy 0];
SForceE = [SFEx SFEy 0];
SForceF = [SFFx SFFy 0];
SForceG = [SFGx SFGy 0];
SInputTorque = [0 0 STin];

staticUnknowns = [SFAx SFAy SFBx SFBy SFCx SFCy SFDx SFDy SFEx SFEy SFFx SFFy SFGx SFGy STin];


syms DFAx DFAy DFBx DFBy DFCx DFCy DFDx DFDy DFEx DFEy DFFx DFFy DFGx DFGy DTin

DForceA = [DFAx DFAy 0];
DForceB = [DFBx DFBy 0];
DForceC = [DFCx DFCy 0];
DForceD = [DFDx DFDy 0];
DForceE = [DFEx DFEy 0];
DForceF = [DFFx DFFy 0];
DForceG = [DFGx DFGy 0];
DInputTorque = [0 0 DTin];

dynamicUnknowns = [DFAx DFAy DFBx DFBy DFCx DFCy DFDx DFDy DFEx DFEy DFFx DFFy DFGx DFGy DTin];
C_prev = C;
E_prev = E;
F_prev = F;




%  360-DEGREE ANALYSIS
for k = 1:nPos

    % Input link AB rotated from its original position.
    currentTheta = initialTheta + thetaRad(k);
    B_new = A + AB*[cos(currentTheta), sin(currentTheta), 0];

   
    % Loop 1: ABCDA
    x0 = B_new(1); y0 = B_new(2); r0 = BC;
    x1 = D(1); y1 = D(2); r1 = CD0;
    dx = x1-x0; dy = y1-y0; d = hypot(dx,dy);

    if d > r0+r1 || d < abs(r0-r1) || d == 0
        fprintf('No valid C position at theta = %.2f degrees\n',thetaDeg(k));
    end

    a = (r0^2-r1^2+d^2)/(2*d);
    h = sqrt(max(0,r0^2-a^2));
    xm = x0 + a*dx/d; ym = y0 + a*dy/d;
    rx = -dy*h/d; ry = dx*h/d;

    C1 = [xm+rx, ym+ry, 0];
    C2 = [xm-rx, ym-ry, 0];

    % Select the solution closest to the previous C position.
    dist1 = norm(C1-C_prev);
    dist2 = norm(C2-C_prev);
    if dist1 < dist2
        C_new = C1;
    else
        C_new = C2;
    end

    % Link 3 is rigid from D through C to E.
    % Therefore E is obtained by extending the D-C line to the full DE
    % length. This enforces the fact that C lies on link DE.
    DC_direction = (C_new-D)/norm(C_new-D);
    E_new = D + DE*DC_direction;

    % Check the expected C-to-E distance.
    if abs(norm(E_new-C_new)-CE0) > 1e-3
        fprintf('DE geometry check is outside tolerance at theta %.2f deg\n', thetaDeg(k));
    end

    % Loop 2: DEFGD
    x1 = G(1); y1 = G(2); r1 = GF;
    dx = x1-x0; dy = y1-y0; d = hypot(dx,dy);

    if d > r0+r1 || d < abs(r0-r1) || d == 0
        fprintf('No valid F position at theta = %.2f degrees\n',thetaDeg(k));
    end

    a = (r0^2-r1^2+d^2)/(2*d);
    h = sqrt(max(0,r0^2-a^2));
    xm = x0 + a*dx/d; ym = y0 + a*dy/d;
    rx = -dy*h/d; ry = dx*h/d;

    F1 = [xm+rx, ym+ry, 0];
    F2 = [xm-rx, ym-ry, 0];

    % Select the solution closest to the previous F position.
    dist1 = norm(F1-F_prev);
    dist2 = norm(F2-F_prev);
    if dist1 < dist2
        F_new = F1;
    else
        F_new = F2;
    end

    % Store positions
    B_pos(k,:) = B_new;
    C_pos(k,:) = C_new;
    E_pos(k,:) = E_new;
    F_pos(k,:) = F_new;


    % Velocity kinematics
    % Loop 1 velocity equation:
    % omega_AB x (B-A) + omega_BC x (C-B)
    % + omega_DE x (D-C) = 0
    M1 = [cross([0,0,1],B_new-A).'; cross([0,0,1],C_new-B_new).'];
    rhs1 = -cross(omega_AB,B_new-A);
    rhs1 = [rhs1(1); rhs1(2)];

    % The DE angular velocity is the second unknown.
    % Solve using x/y components only.
    coeff = [cross([0,0,1],C_new-B_new).', cross([0,0,1],D-C_new).'];
    omegaSol1 = coeff(1:2,1:2)\rhs1;
    omegaBC_k = omegaSol1(1);
    omegaDE_k = omegaSol1(2);

    omegaBC(k) = omegaBC_k;
    omegaDE(k) = omegaDE_k;

    wBC_vec = [0,0,omegaBC_k];
    wDE_vec = [0,0,omegaDE_k];

    vB_k = cross(omega_AB,B_new-A);
    vC_k = vB_k + cross(wBC_vec,C_new-B_new);
    vE_k = cross(wDE_vec,E_new-D);

    % Loop 2 velocity equation:
    % omega_DE x (E-D) + omega_EF x (F-E)
    % + omega_FG x (G-F) = 0
    rhs2 = -cross(wDE_vec,E_new-D);
    coeff2 = [cross([0,0,1],F_new-E_new).', cross([0,0,1],G-F_new).'];
    omegaSol2 = coeff2(1:2,1:2)\[rhs2(1);rhs2(2)];

    omegaEF_k = omegaSol2(1);
    omegaFG_k = omegaSol2(2);

    omegaEF(k) = omegaEF_k;
    omegaFG(k) = omegaFG_k;

    wEF_vec = [0,0,omegaEF_k];
    wFG_vec = [0,0,omegaFG_k];

    vF_k = vE_k + cross(wEF_vec,F_new-E_new);

    vB(k,:) = vB_k;
    vC(k,:) = vC_k;
    vE(k,:) = vE_k;
    vF(k,:) = vF_k;


    %Acceleration Kinematics   
    % Loop 1 acceleration:
    % alpha_AB x rBA + omega_AB x (omega_AB x rBA)
    % + alpha_BC x rCB + omega_BC x (omega_BC x rCB)
    % + alpha_DE x rDC + omega_DE x (omega_DE x rDC) = 0
    aBA = cross(omega_AB,cross(omega_AB,B_new-A));

    aBC_normal = cross(wBC_vec,cross(wBC_vec,C_new-B_new));
    aDE_normal = cross(wDE_vec,cross(wDE_vec,D-C_new));

    rhsA1 = -(aBA + aBC_normal + aDE_normal);

    coeffA1 = [cross([0,0,1],C_new-B_new).', cross([0,0,1],D-C_new).'];
    alphaSol1 = coeffA1(1:2,1:2)\[rhsA1(1);rhsA1(2)];

    alphaBC_k = alphaSol1(1);
    alphaDE_k = alphaSol1(2);

    alphaBC(k) = alphaBC_k;
    alphaDE(k) = alphaDE_k;

    alphaBC_vec = [0,0,alphaBC_k];
    alphaDE_vec = [0,0,alphaDE_k];

    aB_k = aBA;
    aC_k = aB_k + cross(alphaBC_vec,C_new-B_new) + aBC_normal;
    aE_k = cross(alphaDE_vec,E_new-D) + cross(wDE_vec,cross(wDE_vec,E_new-D));


    % Loop 2 acceleration
    aDE_E = cross(alphaDE_vec,E_new-D) + cross(wDE_vec,cross(wDE_vec,E_new-D));

    aEF_normal = cross(wEF_vec,cross(wEF_vec,F_new-E_new));
    aFG_normal = cross(wFG_vec,cross(wFG_vec,G-F_new));

    rhsA2 = -(aDE_E + aEF_normal + aFG_normal);

    coeffA2 = [cross([0,0,1],F_new-E_new).', cross([0,0,1],G-F_new).'];
    alphaSol2 = coeffA2(1:2,1:2)\[rhsA2(1);rhsA2(2)];

    alphaEF_k = alphaSol2(1);
    alphaFG_k = alphaSol2(2);

    alphaEF(k) = alphaEF_k;
    alphaFG(k) = alphaFG_k;

    alphaEF_vec = [0,0,alphaEF_k];
    alphaFG_vec = [0,0,alphaFG_k];

    aF_k = aE_k + cross(alphaEF_vec,F_new-E_new) + aEF_normal;

    aB(k,:) = aB_k;
    aC(k,:) = aC_k;
    aE(k,:) = aE_k;
    aF(k,:) = aF_k;

    % Mass-center locations
    S1 = (A+B_new)/2;
    S2 = (B_new+C_new)/2;
    S3 = (D+E_new)/2;
    S4 = (E_new+F_new)/2;
    S5 = (F_new+G)/2;

    S1_pos(k,:) = S1;
    S2_pos(k,:) = S2;
    S3_pos(k,:) = S3;
    S4_pos(k,:) = S4;
    S5_pos(k,:) = S5;

   
    % Mass-center accelerations
    aS1_k = cross(alpha_AB,S1-A) + cross(omega_AB,cross(omega_AB,S1-A));
    aS2_k = aB_k + cross(alphaBC_vec,S2-B_new) + cross(wBC_vec,cross(wBC_vec,S2-B_new));
    aS3_k = cross(alphaDE_vec,S3-D) + cross(wDE_vec,cross(wDE_vec,S3-D));
    aS4_k = aE_k + cross(alphaEF_vec,S4-E_new) + cross(wEF_vec,cross(wEF_vec,S4-E_new));
    aS5_k = cross(alphaFG_vec,S5-G) + cross(wFG_vec,cross(wFG_vec,S5-G));

    aS1(k,:) = aS1_k;
    aS2(k,:) = aS2_k;
    aS3(k,:) = aS3_k;
    aS4(k,:) = aS4_k;
    aS5(k,:) = aS5_k;


    % Centers of mass

    % Link 1 = AB
    % Link 2 = BC
    % Link 3 = DE (C is a pin located on this rigid link)
    % Link 4 = EF
    % Link 5 = FG
   
    S1 = (A+B_new)/2;
    S2 = (B_new+C_new)/2;
    S3 = (D+E_new)/2;
    S4 = (E_new+F_new)/2;
    S5 = (F_new+G)/2;

  
    %static equilibrium

    % Sum of forces = 0 and sum of moments = 0 for each moving link.
   
    eqnS1 = SForceA + SForceB + WAB == 0;
    eqnS2 = cross(A-S1,SForceA) + cross(B_new-S1,SForceB) + SInputTorque == 0;

    eqnS3 = -SForceB + SForceC + WBC == 0;
    eqnS4 = cross(B_new-S2,-SForceB) + cross(C_new-S2,SForceC) == 0;

    eqnS5 = -SForceC + SForceD - SForceE + WDE == 0;
    eqnS6 = cross(C_new-S3,-SForceC) + cross(D-S3,SForceD) + cross(E_new-S3,-SForceE) == 0;

    eqnS7 = -SForceE + SForceF + WEF == 0;
    eqnS8 = cross(E_new-S4,-SForceE) + cross(F_new-S4,SForceF) == 0;

    eqnS9 = -SForceF + SForceG + WFG + AppliedForce == 0;
    eqnS10 = cross(F_new-S5,-SForceF) + cross(G-S5,SForceG) + cross(P-S5,AppliedForce) == 0;

    staticEqns = [eqnS1(1:2),eqnS2(3),eqnS3(1:2),eqnS4(3), eqnS5(1:2),eqnS6(3),eqnS7(1:2),eqnS8(3), eqnS9(1:2),eqnS10(3)];

    staticSolution = solve(staticEqns,staticUnknowns,'Real',true);

    xStatic = double([staticSolution.SFAx, staticSolution.SFAy, staticSolution.SFBx, staticSolution.SFBy, staticSolution.SFCx, staticSolution.SFCy, staticSolution.SFDx, staticSolution.SFDy, staticSolution.SFEx, staticSolution.SFEy, staticSolution.SFFx, staticSolution.SFFy, staticSolution.SFGx, staticSolution.SFGy, staticSolution.STin]);

    staticForce(k,1,1:2) = xStatic(1:2);
    staticForce(k,2,1:2) = xStatic(3:4);
    staticForce(k,3,1:2) = xStatic(5:6);
    staticForce(k,4,1:2) = xStatic(7:8);
    staticForce(k,5,1:2) = xStatic(9:10);
    staticForce(k,6,1:2) = xStatic(11:12);
    staticForce(k,7,1:2) = xStatic(13:14);
    staticTorque(k) = xStatic(15);

    % DYNAMIC EQUILIBRIUM / NEWTON'S SECOND LAW

  
    eqnD1 = DForceA + DForceB + WAB == MassAB*aS1_k;
    eqnD2 = cross(A-S1,DForceA) + cross(B_new-S1,DForceB) + DInputTorque == J_AB*alpha_AB;

    eqnD3 = -DForceB + DForceC + WBC == MassBC*aS2_k;
    eqnD4 = cross(B_new-S2,-DForceB) + cross(C_new-S2,DForceC) == J_BC*[0 0 alphaBC_k];

    eqnD5 = -DForceC + DForceD - DForceE + WDE == MassDE*aS3_k;
    eqnD6 = cross(C_new-S3,-DForceC) + cross(D-S3,DForceD) + cross(E_new-S3,-DForceE) == J_DE*[0 0 alphaDE_k];

    eqnD7 = -DForceE + DForceF + WEF == MassEF*aS4_k;
    eqnD8 = cross(E_new-S4,-DForceE) + cross(F_new-S4,DForceF) == J_EF*[0 0 alphaEF_k];

    eqnD9 = -DForceF + DForceG + WFG + AppliedForce == MassFG*aS5_k;
    eqnD10 = cross(F_new-S5,-DForceF) + cross(G-S5,DForceG) + cross(P-S5,AppliedForce) == J_FG*[0 0 alphaFG_k];

    dynamicEqns = [eqnD1(1:2),eqnD2(3),eqnD3(1:2),eqnD4(3), eqnD5(1:2),eqnD6(3),eqnD7(1:2),eqnD8(3), eqnD9(1:2),eqnD10(3)];

    dynamicSolution = solve(dynamicEqns,dynamicUnknowns,'Real',true);

    xDynamic = double([dynamicSolution.DFAx, dynamicSolution.DFAy, ...
        dynamicSolution.DFBx, dynamicSolution.DFBy, ...
        dynamicSolution.DFCx, dynamicSolution.DFCy, ...
        dynamicSolution.DFDx, dynamicSolution.DFDy, ...
        dynamicSolution.DFEx, dynamicSolution.DFEy, ...
        dynamicSolution.DFFx, dynamicSolution.DFFy, ...
        dynamicSolution.DFGx, dynamicSolution.DFGy, dynamicSolution.DTin]);

    dynamicForce(k,1,1:2) = xDynamic(1:2);
    dynamicForce(k,2,1:2) = xDynamic(3:4);
    dynamicForce(k,3,1:2) = xDynamic(5:6);
    dynamicForce(k,4,1:2) = xDynamic(7:8);
    dynamicForce(k,5,1:2) = xDynamic(9:10);
    dynamicForce(k,6,1:2) = xDynamic(11:12);
    dynamicForce(k,7,1:2) = xDynamic(13:14);
    dynamicTorque(k) = xDynamic(15);

    % Update branch-selection positions
    B_prev = B_new;
    C_prev = C_new;
    E_prev = E_new;
    F_prev = F_new;
end



fprintf('A = [% .6f, % .6f]\n', A(1),A(2));
fprintf('B = [% .6f, % .6f]\n', B_pos(1,1),B_pos(1,2));
fprintf('C = [% .6f, % .6f]\n', C_pos(1,1),C_pos(1,2));
fprintf('D = [% .6f, % .6f]\n', D(1),D(2));
fprintf('E = [% .6f, % .6f]\n', E_pos(1,1),E_pos(1,2));
fprintf('F = [% .6f, % .6f]\n', F_pos(1,1),F_pos(1,2));
fprintf('G = [% .6f, % .6f]\n', G(1),G(2));


fprintf('AB = % .6f\n', omegaInput);
fprintf('BC = % .6f\n', omegaBC(1));
fprintf('DE = % .6f\n', omegaDE(1));
fprintf('EF = % .6f\n', omegaEF(1));
fprintf('FG = % .6f\n', omegaFG(1));

fprintf('\nAngular accelerations [rad/s^2]:\n');
fprintf('AB = % .6f\n', alpha_AB(3));
fprintf('BC = % .6f\n', alphaBC(1));
fprintf('DE = % .6f\n', alphaDE(1));
fprintf('EF = % .6f\n', alphaEF(1));
fprintf('FG = % .6f\n', alphaFG(1));

fprintf('\nStatic joint forces [N]:\n');
jointNames = {'A','B','C','D','E','F','G'};
for j = 1:7
    fprintf('%s = [% .6f, % .6f]\n',jointNames{j}, ...
        staticForce(1,j,1),staticForce(1,j,2));
end
fprintf('Static input torque = % .6f N-m\n',staticTorque(1));

fprintf('\nDynamic joint forces [N]:\n');
for j = 1:7
    fprintf('%s = [% .6f, % .6f]\n',jointNames{j}, ...
        dynamicForce(1,j,1),dynamicForce(1,j,2));
end
fprintf('Dynamic input torque = % .6f N-m\n',dynamicTorque(1));


fprintf('FIRST POSITION RESULTS (theta = 0 deg)\n');
fprintf('\nJoint positions [m]:\n');
fprintf('A = [% .6f, % .6f]\n', A(1),A(2));
fprintf('B = [% .6f, % .6f]\n', B_pos(1,1),B_pos(1,2));
fprintf('C = [% .6f, % .6f]\n', C_pos(1,1),C_pos(1,2));
fprintf('D = [% .6f, % .6f]\n', D(1),D(2));
fprintf('E = [% .6f, % .6f]\n', E_pos(1,1),E_pos(1,2));
fprintf('F = [% .6f, % .6f]\n', F_pos(1,1),F_pos(1,2));
fprintf('G = [% .6f, % .6f]\n', G(1),G(2));

fprintf('\nAngular velocities [rad/s]:\n');
fprintf('AB = % .6f\n', omegaInput);
fprintf('BC = % .6f\n', omegaBC(1));
fprintf('DE = % .6f\n', omegaDE(1));
fprintf('EF = % .6f\n', omegaEF(1));
fprintf('FG = % .6f\n', omegaFG(1));

fprintf('\nAngular accelerations [rad/s^2]:\n');
fprintf('AB = % .6f\n', alpha_AB(3));
fprintf('BC = % .6f\n', alphaBC(1));
fprintf('DE = % .6f\n', alphaDE(1));
fprintf('EF = % .6f\n', alphaEF(1));
fprintf('FG = % .6f\n', alphaFG(1));

fprintf('\nStatic joint forces [N]:\n');
jointNames = {'A','B','C','D','E','F','G'};
for j = 1:7
    fprintf('%s = [% .6f, % .6f]\n',jointNames{j}, ...
        staticForce(1,j,1),staticForce(1,j,2));
end
fprintf('Static input torque = % .6f N-m\n',staticTorque(1));

fprintf('\nDynamic joint forces [N]:\n');
for j = 1:7
    fprintf('%s = [% .6f, % .6f]\n',jointNames{j}, ...
        dynamicForce(1,j,1),dynamicForce(1,j,2));
end
fprintf('Dynamic input torque = % .6f N-m\n',dynamicTorque(1));


%--------------plots--------------------------------------------------

figure('Name','Initial Kinematic breakdown');
plot([A(1),B_pos(1,1),C_pos(1,1),D(1),A(1)], ...
    [A(2),B_pos(1,2),C_pos(1,2),D(2),A(2)], ...
    '-o','LineWidth',1.5);
hold on;
plot([D(1),C_pos(1,1),E_pos(1,1),F_pos(1,1),G(1),D(1)], ...
    [D(2),C_pos(1,2),E_pos(1,2),F_pos(1,2),G(2),D(2)], ...
    '-o','LineWidth',1.5);
plot(P(1),P(2),'kx','MarkerSize',10,'LineWidth',2);
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('Six-Bar Linkage - First Position');
legend('Loop 1: A-B-C-D','Loop 2: D-C-E-F-G','Artifact load point P', ...
    'Location','best');

text(A(1),A(2),'  A');
text(B_pos(1,1),B_pos(1,2),'  B');
text(C_pos(1,1),C_pos(1,2),'  C');
text(D(1),D(2),'  D');
text(E_pos(1,1),E_pos(1,2),'  E');
text(F_pos(1,1),F_pos(1,2),'  F');
text(G(1),G(2),'  G');



figure('Name','Joint Positions');
plot(thetaDeg,B_pos(:,1),'LineWidth',1.2);
hold on;
plot(thetaDeg,B_pos(:,2),'LineWidth',1.2);
plot(thetaDeg,C_pos(:,1),'LineWidth',1.2);
plot(thetaDeg,C_pos(:,2),'LineWidth',1.2);
plot(thetaDeg,E_pos(:,1),'LineWidth',1.2);
plot(thetaDeg,E_pos(:,2),'LineWidth',1.2);
plot(thetaDeg,F_pos(:,1),'LineWidth',1.2);
plot(thetaDeg,F_pos(:,2),'LineWidth',1.2);
grid on;
xlabel('Input angle [deg]');
ylabel('Position [m]');
title('Joint Positions vs. Input Angle');
legend('B_x','B_y','C_x','C_y','E_x','E_y','F_x','F_y');





figure('Name','Joint Velocities');
plot(thetaDeg,vB(:,1),'LineWidth',1.2);
hold on;
plot(thetaDeg,vB(:,2),'LineWidth',1.2);
plot(thetaDeg,vC(:,1),'LineWidth',1.2);
plot(thetaDeg,vC(:,2),'LineWidth',1.2);
plot(thetaDeg,vE(:,1),'LineWidth',1.2);
plot(thetaDeg,vE(:,2),'LineWidth',1.2);
plot(thetaDeg,vF(:,1),'LineWidth',1.2);
plot(thetaDeg,vF(:,2),'LineWidth',1.2);
grid on;
xlabel('Input angle [deg]');
ylabel('Velocity [m/s]');
title('Joint Velocities vs. Input Angle');
legend('B_x','B_y','C_x','C_y','E_x','E_y','F_x','F_y');

figure('Name','Angular Velocities');
plot(thetaDeg,omegaBC,'LineWidth',1.3);
hold on;
plot(thetaDeg,omegaDE,'LineWidth',1.3);
plot(thetaDeg,omegaEF,'LineWidth',1.3);
plot(thetaDeg,omegaFG,'LineWidth',1.3);
yline(omegaInput,'--');
grid on;
xlabel('Input angle [deg]');
ylabel('Angular velocity [rad/s]');
title('Link Angular Velocities');
legend('BC','DE','EF','FG','Input AB');




figure('Name','Joint Accelerations');
plot(thetaDeg,aB(:,1),'LineWidth',1.2);
hold on;
plot(thetaDeg,aB(:,2),'LineWidth',1.2);
plot(thetaDeg,aC(:,1),'LineWidth',1.2);
plot(thetaDeg,aC(:,2),'LineWidth',1.2);
plot(thetaDeg,aE(:,1),'LineWidth',1.2);
plot(thetaDeg,aE(:,2),'LineWidth',1.2);
plot(thetaDeg,aF(:,1),'LineWidth',1.2);
plot(thetaDeg,aF(:,2),'LineWidth',1.2);
grid on;
xlabel('Input angle [deg]');
ylabel('Acceleration [m/s^2]');
title('Joint Accelerations vs. Input Angle');
legend('B_x','B_y','C_x','C_y','E_x','E_y','F_x','F_y');

figure('Name','Angular Accelerations');
plot(thetaDeg,alphaBC,'LineWidth',1.3);
hold on;
plot(thetaDeg,alphaDE,'LineWidth',1.3);
plot(thetaDeg,alphaEF,'LineWidth',1.3);
plot(thetaDeg,alphaFG,'LineWidth',1.3);
grid on;
xlabel('Input angle [deg]');
ylabel('Angular acceleration [rad/s^2]');
title('Link Angular Accelerations');
legend('BC','DE','EF','FG');





figure('Name','Static Joint Forces');
plot(thetaDeg,staticForceMag,'LineWidth',1.1);
grid on;
xlabel('Input angle [deg]');
ylabel('Force magnitude [N]');
title('Static Joint Force Magnitudes');
legend(jointNames,'Location','best');

figure('Name','Static Input Torque');
plot(thetaDeg,staticTorque,'LineWidth',1.3);
grid on;
xlabel('Input angle [deg]');
ylabel('Torque [N-m]');
title('Static Input Torque');




figure('Name','Dynamic Joint Forces');
plot(thetaDeg,dynamicForceMag,'LineWidth',1.1);
grid on;
xlabel('Input angle [deg]');
ylabel('Force magnitude [N]');
title('Dynamic Joint Force Magnitudes');
legend(jointNames,'Location','best');

figure('Name','Dynamic Input Torque');
plot(thetaDeg,dynamicTorque,'LineWidth',1.3);
grid on;
xlabel('Input angle [deg]');
ylabel('Torque [N-m]');
title('Dynamic Input Torque');



figure('Name','Mass Center Accelerations');
plot(thetaDeg,sqrt(sum(aS1.^2,2)),'LineWidth',1.2);
hold on;
plot(thetaDeg,sqrt(sum(aS2.^2,2)),'LineWidth',1.2);
plot(thetaDeg,sqrt(sum(aS3.^2,2)),'LineWidth',1.2);
plot(thetaDeg,sqrt(sum(aS4.^2,2)),'LineWidth',1.2);
plot(thetaDeg,sqrt(sum(aS5.^2,2)),'LineWidth',1.2);
grid on;
xlabel('Input angle [deg]');
ylabel('Acceleration magnitude [m/s^2]');
title('Mass-Center Acceleration Magnitudes');
legend('AB','BC','DE','EF','FG','Location','best');

fprintf('\nAnalysis complete. All 360 positions were processed.\n');