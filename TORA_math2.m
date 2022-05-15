in = 1; %Open loop control input

M = 1.3608;    %[kg] -> cart mass
m = 0.096;     %[kg] -> proof mass
J = 0.0002175; %[kgm^2] -> mass moment of inertia
k = 186.3;     %[N/m] -> spring constant
Len = 2.5;     %[m] -> armature length

xbar = [0 0 0 0]';       %Equilibrium conditions
x0 = [0 0 0.01 0.01]';   %Initial Conditions

% Cacluating the Linearised A and B Matricies using the jacobian function
syms x1 x2 x3 x4 u

% Nonlinear System
dx1 = (((M+m)*(J+m*(Len^2)))-((m^2)*(Len^2)*(cos(x1)^2)));   
x2_dot = (1/dx1)*((M+m)*u-m*Len*cos(x1)*(m*Len*(x2^2)*sin(x1)-k*x3));
x4_dot = (1/dx1)*(-m*Len*u*cos(x1)+(J+m*(Len^2))*(m*Len*(x2^2)*sin(x1)-k*x3));

x_bar = [0 0 0 0]; 
vars = [x1 x2 x3 x4];
f = [x2, x2_dot, x4, x4_dot];
JacobianA = jacobian(f, vars);
A = double(subs(JacobianA, vars, x_bar));

jacobianB = jacobian(f, u);
B = double(subs(jacobianB, vars, x_bar));

C = [1 0 0 0;
     0 0 1 0];
D = [0 0]';