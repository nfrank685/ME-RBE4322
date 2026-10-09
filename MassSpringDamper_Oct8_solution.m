%solving diff eq generated using bond graph technique

clc
clear

%initial conditions

x0 = [80 0]; %initial condition for momentum and displacement

timespan = [0 10]; %0 to 10 seconds

[t,x] = ode45(@MassSpringDamper_Oct8,timespan,x0);

plot(t,x(:,1))

plot(t,x(:,2))