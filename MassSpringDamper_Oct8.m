function xprime = MassSpringDamper_Oct8(t,x)

%all parameters and eqs are stored
M=8; %kg
K=100; %N/m
D=50; %Ns/m
F=50; %N

xprime = [F - (D/M)*x(1) - K*x(2); (1/M)*x(1)];

end