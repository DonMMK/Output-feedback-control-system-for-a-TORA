%% EGH445 Modern Control TORA Model
% Discrete Time Modelling and Control with Linear systems
% Author: Don Kaluarachchi
% Acknowledgements: Andrew R.
clearvars;
clear all;
close all;
clc;

%% Parameters

M = 1.3608;    % kg Mass of Translating Oscilattor
m = 0.096 ;    % kg Mass of Rotating Actuator
L = 1     ;    % m  Length of Rotating Actuator
J = 0.0002175; % kg m2 Intertia
k = 186.3;     % N/m Spring Constant

%% Define State Space Matrices Equilibrium A:

% linearised model equlibrium A
A=[0, 1, 0, 0; 
    0, 0, ((m*L*k)/((J+(m*L^2))*(m+M)-((m^2)*(L^2)))), 0;
    0, 0, 0, 1; 
    0, 0, ((-k*(J+(m*L^2)))/((J+(m*L^2))*(m+M)-(m^2*L^2))), 0];
%Aa=[0 0 1 0;0 0 0 1;0 -m*g/(Mc) 0 0;0 g*(Mc+m)/(l*Mc) 0 0]; %no damping
B=[0;
    (m+M/((J+(m*L^2))*(m+M)-(m^2*L^2)));
    0;
    ((-m*L)/((J+m*L^2)*(m+M)-(m^2*L^2)))];

C=eye(4); 

D=zeros(4,1);

x_bar = [0 0 0 0]'; %equlibrium point
x0 =[20*pi/180 , 0 ,  0.1 , 0]'; %Initial condition
x0_tilde = x0 - x_bar; %linearised initial condition


%% Create Continuous time model

%Create State Space object in continuous time:
sysc=ss(A,B,C,D);

%% Discretisation

%Sampling Time
Ts=0.01; % Use 0.001
%Ts = 0.05; %unstable sampled-data!

%Convert continuous to discrete using c2d and the Zero-order-hold method
sysdzoh=c2d(sysc,Ts,'zoh');

%Extract Discretised State Space Matrices
Ad=sysdzoh.A;
Bd=sysdzoh.B;
Cd=sysdzoh.C;
Dd=sysdzoh.D;

use_approximation = 1;      %turn on/off use of functions matexp intmatexpB

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
% assert(test,strcat('Error System is Not controllable at Point',point))
if(test == 1)
    fprintf('Equilibrium point Controllable')
else
    fprintf('Not controllable ://')
end
%Compute Controller gains
%p = [-3 -4 -5 -6]; %Stable Poles in continuous time s-domain

% Pole Placement
% Required Settling Time
Required_Settlting_Time = 4;  

% Percentage Overshoot
Percentage_Overshoot = 2; 

% Poles
zeta_numerator = -log(Percentage_Overshoot/100);
zeta_denominator = sqrt(pi^2+log(Percentage_Overshoot/100)^2);
zeta = zeta_numerator / zeta_denominator;

wn = 4/(Required_Settlting_Time*zeta);

% Dominant pole near the jw axis -> slow response
% 2 fast 2 slow

%Pole 1 and 2 
Poles_1_2 = roots([1, 2*zeta*wn, wn^2]) 
%Pole 3 and 4 
Pole_3 = 7 * Poles_1_2(1)
Pole_4 = 7 * Poles_1_2(2)

%Stable Poles in S domain
p = [Poles_1_2(1) ,Poles_1_2(2), Pole_3, Pole_4];


p_z = exp(p*Ts);   %Map to Poles in discrete time z-domain
Kd = place(Ad,Bd,p_z);
% Kd(3) = -0.9223; % 

%% Simulation Parameters
stoptime = 4;               %Run simulation in seconds
DT_Lin_animation = 0;       %turn on/off animation of Discrete Time model
SD_Lin_animation = 0;       %turn on/off animation of Sampled Data model

%% Run simulations

%ODE solver settings:
h = 0.001;

%Run simulation of Discrete Time model and Sampled Data model
DT_Lin = sim('DT_Lin_TORA','Solver','ode4','FixedStep','h','StopTime','stoptime');
SD_Lin = sim('SD_Lin_TORA','Solver','ode4','FixedStep','h','StopTime','stoptime');

%Run simulation of Discrete Time model and Sampled Data model
DT_NL = sim('DT_NL_TORA','Solver','ode4','FixedStep','h','StopTime','stoptime');
SD_NL = sim('SD_NL_TORA','Solver','ode4','FixedStep','h','StopTime','stoptime');

% %% Plot Results: Andrews plot code
% 
% %Plot states and estimates over time for both models in one figure:
% figure
% labels = {'Cart Position [m]','Pendulum angle [deg]',...
%     'Cart Velocity [m/s]','Pendulum Rate [deg/s]','Control Force [N]'};
% r2d = [1, 180/pi, 1, 180/pi]; %Conversions from radians to degrees
% sgtitle(strcat('Design Using Linearisation about Equilibrium Point:',point))
% for ii=1:4
%     subplot(5,1,ii)
%     stem(DT_Lin.t,r2d(ii)*DT_Lin.x(:,ii),'b')
%     hold on
%     plot(SD_Lin.t,r2d(ii)*SD_Lin.x(:,ii),'r','LineWidth',2)
%     xlabel('time [s]')
%     ylabel(labels{ii})
%     legend(strcat('x_',num2str(ii),' Discrete-time model'),...
%         strcat('x_',num2str(ii),' sampled-data model'))
%     grid on
% end
% 
% %Control Force over time:
% subplot(5,1,5)
% stem(DT_Lin.t,DT_Lin.F,'b')
% hold on
% plot(SD_Lin.t,SD_Lin.F,'r','LineWidth',2)
% legend('F Discrete-time model','F sampled-data model')
% xlabel('time [s]')
% ylabel(labels{5})
% grid on
% 
% %% Run Animation
% %NOTE inputs are: (time vector, x1, x2, equilibrium x1, equilibrium x2)
% if(DT_Lin_animation==true)
%     Cart_Pendulum_Animation(DT_Lin.t,DT_Lin.x(:,1)+x_bar(1),...
%         DT_Lin.x(:,2)+x_bar(2),x_bar(1),x_bar(2))    
% end
% if(SD_Lin_animation==true)
%     Cart_Pendulum_Animation(SD_Lin.t,SD_Lin.x(:,1)+x_bar(1),...
%         SD_Lin.x(:,2)+x_bar(2),x_bar(1),x_bar(2))
% end

%% Plot 1
r2d = [180/pi,1]; %Conversions from radians to degrees


figure('Name',' Discrete Time Model (Linear)');
subplot(3,2,1:2)
stem(DT_Lin.t,DT_Lin.F,'b')
hold on
plot(SD_Lin.t,SD_Lin.F,'r','LineWidth',2)
grid on
xlabel('Time(s)')
ylabel('Force(N)')
title ('Control Force')
legend('F Discrete-time model','F Sampled-data model')


subplot(3,2,3)
stem(DT_Lin.t,r2d(1)*DT_Lin.x(:,1),'b')
hold on
plot(SD_Lin.t,r2d(1)*SD_Lin.x(:,1),'r','LineWidth',2)
xlabel('Time(s)')
ylabel('Angle(deg)')
title ('State x1: Angle of rotating actuator')
legend('x1 Discrete-Time Model','x1 Sampled-Data Model','location','northeast')


subplot(3,2,4)
stem(DT_Lin.t,r2d(1)*DT_Lin.x(:,2),'b')
hold on
plot(SD_Lin.t,r2d(1)*SD_Lin.x(:,2),'r','LineWidth',2)
xlabel('Time(s)')
ylabel('Angular Velocity(deg/s)')
title ('State x2: Angular velocity of rotating actuator')
legend('x2 Discrete-Time Model','x2 Sampled-Data Model','location','northeast')


subplot(3,2,5)
stem(DT_Lin.t,r2d(2)*DT_Lin.x(:,3),'b')
hold on
plot(SD_Lin.t,r2d(2)*SD_Lin.x(:,3),'r','LineWidth',2)
xlabel('Time(s)')
ylabel('Position(m)')
title ('State x3: Position of Translational oscillator')
legend('x3 Discrete-Time Model','x3 Sampled-Data Model','location','northeast')

subplot(3,2,6)
stem(DT_Lin.t,r2d(2)*DT_Lin.x(:,4),'b')
hold on
plot(SD_Lin.t,r2d(2)*SD_Lin.x(:,4),'r','LineWidth',2)
xlabel('Time(s)')
ylabel('Velocity')
title ('State x4: Velocity of Translational oscillator')
legend('x4 Discrete-Time Model','x4 Sampled-Data Model','location','northeast')

%% Plot 2
r2d = [180/pi,1]; %Conversions from radians to degrees


figure('Name',' Discrete Time Model (Non-Linear)');
subplot(3,2,1:2)
stem(DT_NL.t,DT_NL.F,'b')
hold on
plot(SD_NL.t,SD_NL.F,'r','LineWidth',2)
grid on
xlabel('Time(s)')
ylabel('Force(N)')
title ('Control Force')
legend('F Discrete-time model','F Sampled-data model')


subplot(3,2,3)
stem(DT_NL.t,r2d(1)*DT_NL.x(:,1),'b')
hold on
plot(SD_NL.t,r2d(1)*SD_NL.x(:,1),'r','LineWidth',2)
xlabel('Time(s)')
ylabel('Angle(deg)')
title ('State x1: Angle of rotating actuator')
legend('x1 Discrete-Time Model','x1 Sampled-Data Model','location','northeast')


subplot(3,2,4)
stem(DT_NL.t,r2d(1)*DT_NL.x(:,2),'b')
hold on
plot(SD_NL.t,r2d(1)*SD_NL.x(:,2),'r','LineWidth',2)
xlabel('Time(s)')
ylabel('Angular Velocity(deg/s)')
title ('State x2: Angular velocity of rotating actuator')
legend('x2 Discrete-Time Model','x2 Sampled-Data Model','location','northeast')


subplot(3,2,5)
stem(DT_NL.t,r2d(2)*DT_NL.x(:,3),'b')
hold on
plot(SD_NL.t,r2d(2)*SD_NL.x(:,3),'r','LineWidth',2)
xlabel('Time(s)')
ylabel('Position(m)')
title ('State x3: Position of Translational oscillator')
legend('x3 Discrete-Time Model','x3 Sampled-Data Model','location','northeast')

subplot(3,2,6)
stem(DT_NL.t,r2d(2)*DT_NL.x(:,4),'b')
hold on
plot(SD_NL.t,r2d(2)*SD_NL.x(:,4),'r','LineWidth',2)
xlabel('Time(s)')
ylabel('Velocity')
title ('State x4: Velocity of Translational oscillator')
legend('x4 Discrete-Time Model','x4 Sampled-Data Model','location','northeast')
