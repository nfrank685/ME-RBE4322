clc
clear

%values of different parameters

M = 3.9068; %kg
K = 39390.87; %N/m
B = 1838.275; %N.s/m
F = 756.198; %N

syms x(t) %x(t) is the x in Mx'' to Bx' to Kx = F

eqn1 = M*diff(x,t,2) + B*diff(x,t) + K*x == F;
%Note in the above equation, x'' is written as diff(x,t,2)
%x' is diff(x,t)

Dx = diff(x,t);

%specifying initial conditions
initialConditions = [x(0) == 0, Dx(0) == 0];

%solving for X(t)
solutionX(t) = dsolve(eqn1, initialConditions);

%solving for V(t) - Velocity as function of time
solutionV = diff(solutionX);

%solving for A(t) - acceleration as function of time
solutionA = diff(solutionV);


%plotting X(t) between times 0 and 10s
figure(1);
fplot(solutionX,[0 5]);
xlabel("time");
ylabel("position");
title("position vs time");

figure(2);
fplot(solutionV, [0 5]);
xlabel("time");
ylabel("velocity");
title("velocity vs time");

figure(3);
fplot(solutionA, [0 5]);
xlabel("time");
ylabel("acceleration");
title("acceleration vs time");




