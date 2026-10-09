%solving diff eq generated using bond graph technique

clc
clear

%initial conditions

x0 = [80 0]; %initial condition for momentum and displacement

timespan = [0 10]; %0 to 10 seconds

[t,x] = ode45(@MassSpringDamper_Oct8,timespan,x0);

figure(1);
plot(t,x(:,1))
title('Momentum over time');
xlabel('Time (s)');
ylabel('Momentum');
title('Momentum Response (kg*m/s)');
grid on;

figure(2);
plot(t,x(:,2))
title('Displacement over time');
xlabel('Time(s)');
ylabel('DIsplacement (m)')
grid on;
