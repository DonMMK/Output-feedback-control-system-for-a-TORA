%% EGH445 Modern Control Cart Pendulum Model
% A. Razjigaev | QUT 2022 
clearvars;
close all;
clc;

%% Simulation Parameters
stoptime = 5;               %Run simulation in seconds
equilibrium_a = 1;          %Choose equilibrium: 1 = point A, 0 = point B
b = 1;                      %turn on/off damping coefficent 0 = no damping
controller_enabled = 1;     %turn on/off controller
use_state_estimates = 1;    %turn on/off state estimate feedback
Lin_animation = 0;          %turn on/off animation of Linear model
NL_animation = 1;           %turn on/off animation of Nonlinear model

%% Define Model Parameters:
g=9.81;
l=0.2;
m=0.15;
Mc=0.4;

%% Define State Space Matrices Equilibrium A:

% linearised model equlibrium A
Aa=[0 0 1 0; 
    0 0 0 1;
    0 -m*g/(Mc) -b/Mc 0;
    0 g*(Mc+m)/(l*Mc) b/(Mc*l) 0];
%Aa=[0 0 1 0;0 0 0 1;0 -m*g/(Mc) 0 0;0 g*(Mc+m)/(l*Mc) 0 0]; %no damping
Ba=[0;0;1/Mc;-1/(l*Mc)];
Ca=eye(2,4); % Position of the cart and angle of the pendulum
Da=zeros(2,1);

xa_bar = [0 0 0 0]'; %equlibrium point
x0a =[0.2 20*pi/180 0 0]'; %Initial condition

%% Define State Space Matrices Equilibrium B:

% linearised model equlibrium B
Ab=[0 0 1 0;
    0 0 0 1;
    0 -m*g/(Mc) -b/Mc 0;
    0 -g*(Mc+m)/(l*Mc) -b/(Mc*l) 0];
%Ab=[0 0 1 0;0 0 0 1;0 -m*g/(Mc) 0 0;0 -g*(Mc+m)/(l*Mc) 0 0]; %no damping
Bb=[0;0;1/Mc;1/(l*Mc)];
Cb=eye(2,4); % Position of the cart and angle of the pendulum
Db=zeros(2,1);

xb_bar = [0 pi 0 0]'; %equlibrium point
x0b =[0.2 200*pi/180 0 0]'; %Initial condition

%% Choose Linear System
if equilibrium_a==true
    %run equilibrium point A simulation
    A = Aa; B = Ba; C = Ca; D = Da;
    x0 = x0a; x_bar = xa_bar; point = ' A';
else
    %run equilibrium point B simulation
    A = Ab; B = Bb; C = Cb; D = Db;
    x0 = x0b; x_bar = xb_bar; point = ' B';
end

%% Controller Design

%Check Controllability
test = rank(ctrb(A,B))==length(A); %via rank
%test = det(ctrb(A,B))~=0; %via determinant (works only with a square matrix)
assert(test,strcat('Error System is Not controllable at Point',point))

%Compute Controller gains
p = [-3 -4 -5 -6]; %Stable Poles for Pole placement
K = place(A,B,p);

%% Observer Design

%Check Observability
test = rank(obsv(A,C))==length(A); %via rank
%test = det(obsv(A,C))~=0; %via determinant (works only with a square matrix)
assert(test,strcat('Error System is Not Observable at Point',point))

%Compute Observer gains
p = [-63 -64 -65 -66]; %Faster than controller poles
L = place(A',C',p)'; %Closed Loop A'- C'L' == A - BK

%% Run simulations

%ODE solver settings:
h = 0.02;

%Run simulation
Lin = sim('Lin_cartmodel','Solver','ode4','FixedStep','h','StopTime','stoptime');
NL = sim('NL_cartmodel','Solver','ode4','FixedStep','h','StopTime','stoptime');

%% Plot Results:

%Plot states and estimates over time for both models in one figure:
figure
labels = {'Cart Position [m]','Pendulum angle [deg]',...
    'Cart Velocity [m/s]','Pendulum Rate [deg/s]','Control Force [N]'};
r2d = [1, 180/pi, 1, 180/pi]; %Conversions from radians to degrees
sgtitle(strcat('Design Using Linearisation about Equilibrium Point:',point))
for ii=1:4
    subplot(5,1,ii)
    plot(NL.t,r2d(ii)*NL.x(:,ii),'b-',...
        NL.t,r2d(ii)*NL.x_hat(:,ii),'g-',...
        Lin.t,r2d(ii)*Lin.x(:,ii),'r--',...
        Lin.t,r2d(ii)*Lin.x_hat(:,ii),'k--','LineWidth',2)
    grid on
    xlabel('time [s]')
    ylabel(labels{ii})
    legend(strcat('x_',num2str(ii),' Non-linear'),...
        strcat('x_',num2str(ii),' hat Non-Linear'),...
        strcat('x_',num2str(ii),' Linearised'),...
        strcat('x_',num2str(ii),' hat Linearised'))
end

%Control Force over time:
subplot(5,1,5)
plot(NL.t,NL.F,'b-',Lin.t,Lin.F,'r--','LineWidth',2)
grid on
xlabel('time [s]')
ylabel(labels{5})
legend('F Non-linear','F linearised')

%% Run Animation
%NOTE inputs are: (time vector, x1, x2, equilibrium x1, equilibrium x2)
if(Lin_animation==true)
    Cart_Pendulum_Animation(Lin.t,Lin.x(:,1),Lin.x(:,2),x_bar(1),x_bar(2))    
end
if(NL_animation==true)
    Cart_Pendulum_Animation(NL.t,NL.x(:,1),NL.x(:,2),x_bar(1),x_bar(2))
end
