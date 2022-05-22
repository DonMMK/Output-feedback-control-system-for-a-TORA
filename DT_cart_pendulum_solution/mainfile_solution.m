%% EGH445 Modern Control Cart Pendulum Model
% Discrete Time Modelling and Control with Linear systems
% Author: Andrew Razjigaev 2022
clearvars;
close all;
clc;

%% Simulation Parameters
stoptime = 3;               %Run simulation in seconds
equilibrium_a = 1;          %Choose equilibrium: 1 = point A, 0 = point B
b = 0;                      %turn on/off damping coefficent 0 = no damping
use_approximation = 1;      %turn on/off use of functions matexp intmatexpB
DT_Lin_animation = 0;       %turn on/off animation of Discrete Time model
SD_Lin_animation = 0;       %turn on/off animation of Sampled Data model

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
Ca=eye(4); 
Da=zeros(4,1);

xa_bar = [0 0 0 0]'; %equlibrium point
x0a =[0.2 20*pi/180 0 0]'; %Initial condition
x0a_tilde = x0a - xa_bar; %linearised initial condition

%% Define State Space Matrices Equilibrium B:

% linearised model equlibrium B
Ab=[0 0 1 0;
    0 0 0 1;
    0 -m*g/(Mc) -b/Mc 0;
    0 -g*(Mc+m)/(l*Mc) -b/(Mc*l) 0];
%Ab=[0 0 1 0;0 0 0 1;0 -m*g/(Mc) 0 0;0 -g*(Mc+m)/(l*Mc) 0 0]; %no damping
Bb=[0;0;1/Mc;1/(l*Mc)];
Cb=eye(4); % Position of the cart and angle of the pendulum
Db=zeros(4,1);

xb_bar = [0 pi 0 0]'; %equlibrium point
x0b =[0.2 200*pi/180 0 0]'; %Initial condition
x0b_tilde = x0b - xb_bar; %linearised initial condition

%% Choose Linear System
if equilibrium_a==true
    %run equilibrium point A simulation
    A = Aa; B = Ba; C = Ca; D = Da;
    x0_tilde = x0a_tilde; x_bar = xa_bar; point = ' A';
else
    %run equilibrium point B simulation
    A = Ab; B = Bb; C = Cb; D = Db;
    x0_tilde = x0b_tilde; x_bar = xb_bar; point = ' B';
end

%% Create Continuous time model

%Create State Space object in continuous time:
sysc=ss(A,B,C,D);

%% Discretisation

%Sampling Time
Ts=0.03; 
%Ts = 0.05; %unstable sampled-data!

%Convert continuous to discrete using c2d and the Zero-order-hold method
sysdzoh=c2d(sysc,Ts,'zoh');

%Extract Discretised State Space Matrices
Ad=sysdzoh.A;
Bd=sysdzoh.B;
Cd=sysdzoh.C;
Dd=sysdzoh.D;

%Use Approximation Functions
n = 4; %order of approximation
if (use_approximation==true)
    Ad=matexp(A,Ts,n);
    Bd=intmatexpB(A,B,Ts,n); 
end

%% Discrete-time Controller Design

%Check Controllability
test = rank(ctrb(Ad,Bd))==length(Ad); %via rank
%test = det(ctrb(A,B))~=0; %via determinant (works only with a square matrix)
assert(test,strcat('Error System is Not controllable at Point',point))

%Compute Controller gains
p = [-3 -4 -5 -6]; %Stable Poles in continuous time s-domain
p_z = exp(p*Ts);   %Map to Poles in discrete time z-domain
Kd = place(Ad,Bd,p_z);

%% Run simulations

%ODE solver settings:
h = 0.001;

%Run simulation of Discrete Time model and Sampled Data model
DT_Lin = sim('DT_Lin_cartmodel','Solver','ode4','FixedStep','h','StopTime','stoptime');
SD_Lin = sim('SD_Lin_cartmodel','Solver','ode4','FixedStep','h','StopTime','stoptime');

%% Plot Results:

%Plot states and estimates over time for both models in one figure:
figure
labels = {'Cart Position [m]','Pendulum angle [deg]',...
    'Cart Velocity [m/s]','Pendulum Rate [deg/s]','Control Force [N]'};
r2d = [1, 180/pi, 1, 180/pi]; %Conversions from radians to degrees
sgtitle(strcat('Design Using Linearisation about Equilibrium Point:',point))
for ii=1:4
    subplot(5,1,ii)
    stem(DT_Lin.t,r2d(ii)*DT_Lin.x(:,ii),'b')
    hold on
    plot(SD_Lin.t,r2d(ii)*SD_Lin.x(:,ii),'r','LineWidth',2)
    xlabel('time [s]')
    ylabel(labels{ii})
    legend(strcat('x_',num2str(ii),' Discrete-time model'),...
        strcat('x_',num2str(ii),' sampled-data model'))
    grid on
end

%Control Force over time:
subplot(5,1,5)
stem(DT_Lin.t,DT_Lin.F,'b')
hold on
plot(SD_Lin.t,SD_Lin.F,'r','LineWidth',2)
legend('F Discrete-time model','F sampled-data model')
xlabel('time [s]')
ylabel(labels{5})
grid on

%% Run Animation
%NOTE inputs are: (time vector, x1, x2, equilibrium x1, equilibrium x2)
if(DT_Lin_animation==true)
    Cart_Pendulum_Animation(DT_Lin.t,DT_Lin.x(:,1)+x_bar(1),...
        DT_Lin.x(:,2)+x_bar(2),x_bar(1),x_bar(2))    
end
if(SD_Lin_animation==true)
    Cart_Pendulum_Animation(SD_Lin.t,SD_Lin.x(:,1)+x_bar(1),...
        SD_Lin.x(:,2)+x_bar(2),x_bar(1),x_bar(2))
end
