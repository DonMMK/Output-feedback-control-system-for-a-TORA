%% EGH445 Modern Control TORA System %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author:
% Submission Date:
% Acknowledgements

%% Reset Workspace to zero
close all; clear all ; clc

%% Parameters

M = 1.3608;    % kg Mass of Translating Oscilattor
m = 0.096 ;    % kg Mass of Rotating Actuator
L = 1     ;    % m  Length of Rotating Actuator
J = 0.0002175; % kg m2 Intertia
k = 186.3;     % N/m Spring Constant

%% Equilibrium Points

% syms x1 x3
% u = 0;
% x2 = 0;
% x4 = 0;
% 
% eq1Num = (m + M)*u - m*l*cos(x1) * (m*l*x2^2*sin(x1) - k*x3);
% eq2Num = -m*l*u*cos(x1) + (J + m*l^2) * (m*l*x2^2*sin(x1) - k*x3);
% delTheta = (J + m*l^2)*(m + M) - m^2*l^2*cos(x1)^2;
% 
% eq1 = eq1Num/delTheta;
% eq2 = eq2Num/delTheta;
% 
% [x1, x3] = solve(eq1 == 0, eq2 == 0);
% x1 = double(x1); x3 = double(x3);
% 
% xa_bar = [ x1(2); x2; x3(2); x4 ];
% xb_bar = [ x1(1); x2; x3(1); x4 ];
% 
% clear x1 x2 x3 x4


%% Stability
% For non Linear systems examine stability of individual equilibrium points
% ASK FOR CLARIFICATION: 

% When the equilibrium point is at 0 - Point A
Aa = [0 , 1 , 0 ,0 ; 
   0 , 0 , ((m*L*k)/((J+(m*L^2))*(m+M)-((m^2)*(L^2)))) , 0;
   0 ,0 ,0 ,1;
   0 , 0, ((-k*(J+(m*L^2)))/((J+(m*L^2))*(m+M)-(m^2*L^2))) , 0;]

EigValues_A = eig(Aa)
if (sum(EigValues_A < 0) >= 1)
    fprintf("This Equilibrium Point with eig value %d is Stable\n" , EigValues_A)
elseif (sum(EigValues_A == 0) >= 1)
   fprintf("This Equilibrium Point with eig value %d is Marginally Stable\n" , EigValues_A) 
else
   fprintf("This Equilibrium Point with eig value %d is Unstable\n" , EigValues_A)
end

% ASK FOR CLARIFICATION: is it meant to be marginally stable

% When the equilibrium point is at 180 - Point B
Ab = [0 , 1 , 0, 0;
    0 ,0 , (-m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) ) , 0;
    0 ,0 ,0 , 1;
    0 , 0, (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  ) , 0;]

EigValues_B = eig(Ab)
if (sum(EigValues_B < 0) >= 1)
    fprintf("This Equilibrium Point with eig value %d is Stable\n" , EigValues_B)
elseif (sum(EigValues_B == 0) >= 1)
   fprintf("This Equilibrium Point with eig value %d is Marginally Stable\n" , EigValues_B) 
else
   fprintf("This Equilibrium Point with eig value %d is Unstable\n" , EigValues_B)
end

% Equilibrium Point A
Ba = [0;
               (m+M/((J+(m*L^2))*(m+M)-(m^2*L^2))) ;
               0 ;
               ((-m*L)/((J+m*L^2)*(m+M)-(m^2*L^2))) ];

% Equilibrium Point B
Bb = [0;
               (m+M/((J+(m*L^2))*(m+M)-(m^2*L^2)))  ;
               0 ;
               ((+m*L)/((J+m*L^2)*(m+M)-(m^2*L^2))) ];

% The C matrix has not been evaluated conclusively as the states that are
% to be observed havent been selected yet.
% C = eye(4)

% The D matrix is zero for this system
% D = zeros(2,2)

%% Pole Placement

% Required Settling Time
Required_Settlting_Time = 4;  % ASK FOR CLARIFICATION:

% Percentage Overshoot
Percentage_Overshoot = 2; % ASK FOR CLARIFICATION:

% Equilibrium Point for the system
x_bar = [0 , 0 , 0 , 0]';

% Starting position of the system
ctoRadians = pi/180;
x0 = [20*(ctoRadians) , 0 , 0.1, 0]';

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
Pole_3 = 9 * Poles_1_2(1)
Pole_4 = 9 * Poles_1_2(2)



           
%% Control Design using state feedback
% Controllability Matrix

C_Aa_Ba = [ Ba , Aa * Ba ,  (Aa^2) * Ba ,  (Aa^3) * Ba] ;
Controllablity_Point_a = ctrb(Aa , Ba)

C_Ab_Bb = [ Bb , Ab * Bb ,  (Ab^2) * Bb ,  (Ab^3) * Bb] ;
Controllablity_Point_b = ctrb(Ab , Bb)

% Calculate the rank of the system to check if system is controllable
Rank_at_Point_a = rank(Controllablity_Point_a)
Rank_at_Point_b = rank(Controllablity_Point_b)

if (Rank_at_Point_a == length(Aa) ) 
    fprintf("At point A matrix is full rank, therefore controllable\n")
else
    fprintf("Not full rank. The rank is: %d " ,Rank_at_Point_a )
end

if (Rank_at_Point_b == length(Ab) ) 
    fprintf("At point B matrix is full rank, therefore controllable\n")
else
    fprintf("Not full rank. The rank is: %d " ,Rank_at_Point_b )
end

% Only Equilibrium Point A will be considered because
% ASK FOR CLARIFICATION:

Pole_P = [Poles_1_2(1) , Poles_1_2(2), Pole_3 , Pole_4 ]

% Gain using matlab place
K = place(Aa , Ba , Pole_P)


%% Control Design using output feedback 

% New C and D matrices
C = [1 , 0 , 0 , 0;
     0 , 0 , 1 , 0]

D = zeros(2,1)

% Calculate observerbility of the matrix
O_Aa_Ca = [C; C*Aa; C*Aa*Aa ; C*Aa * Aa* Aa]
Observability_Point_a = obsv(Aa,C)

% Calculate the rank of the system to check if system is observable
Rank2_at_Point_a = rank(Observability_Point_a)

if (Rank2_at_Point_a == 4 ) 
    fprintf( "full rank, therefore observable")
else
    fprintf("Not full rank. The rank is: %d " ,Rank2_at_Point_a )
end

desiredPoles = [-65 , -65 , -165 , -165];
L_Value = place(Aa', C', desiredPoles)'; % ASK FOR CLARIFICATION: place vs acker

Observer_In = [x_bar(1); x_bar(3)] % ASK FOR CLARIFICATION:


%% Other parameters to set before simulations

stop_time = 10;
stateEstimates = 1;
Boolean_Flag_for_Controller = 1;

% ODE Solver step size
h = 0.01;

%% Simulation 1: Standard Simulation

NL = sim('TORA_Non_Linear', 'Solver','ode4','FixedStep','h','StopTime','stop_time');
Lin = sim('TORA_Linear','Solver','ode4','FixedStep','h','StopTime','stop_time' );

%% Simulation 2: Simulation without the controller

Boolean_Flag_for_Controller = 0;
NL_No_C = sim('TORA_Non_Linear', 'Solver','ode4','FixedStep','h','StopTime','stop_time');
L_No_C = sim('TORA_Linear','Solver','ode4','FixedStep','h','StopTime','stop_time' );

%% Simulation 3: Starting Conditions varied to test controller capabilities

x0 = [10*pi/180 0 -1 0]';
Boolean_Flag_for_Controller = 1;
NL_Hard_1 = sim('TORA_Non_Linear', 'Solver','ode4','FixedStep','h','StopTime','stop_time');
Lin_Hard_1 = sim('TORA_Linear','Solver','ode4','FixedStep','h','StopTime','stop_time' );



%% Plot 1
r2d = [180/pi,1];
figure('Name',' Uncontrolled vs Controlled response: Non Linear');
%Control Force 
subplot(3,2,1:2)

plot(NL.t,NL.F,'b', Lin.t,Lin.F,'r--')
grid on
xlabel('Time(s)')
ylabel('Force(N)')
title ('Control Force')
legend('Non-linear','Linear')

% For state x1
subplot(3,2,3)
plot(Lin.t,r2d(1)*Lin.x(:,1),'g-', ...
    Lin.t,r2d(1)*Lin.x_hat(:,1),'k--',...
    NL.t,r2d(1)*NL.x(:,1),'r',...
    NL.t,r2d(1)*NL.x_hat(:,1),'b--')
grid on
xlabel('Time(s)')
ylabel('Angle(deg)')
title ('State x1: Angle of rotating actuator')
legend('x1 Linear','x1 hat Linear','x1 Non-Linear','x1 hat Non-Linear','location','northeast')


% For state x2
subplot(3,2,4)
plot(Lin.t,r2d(1)*Lin.x(:,2),'g',...
    Lin.t,r2d(1)*Lin.x_hat(:,2),'k--',...
    NL.t,r2d(1)*NL.x(:,2),'r',...
    NL.t,r2d(1)*NL.x_hat(:,2),'b--')
grid on
xlabel('Time(s)')
ylabel('Angular Velocity(deg/s)')
title ('State x2: Angular velocity of rotating actuator')
legend('x2 Linear','x2 hat Linear','x2 Non-Linear','x2 hat Non-Linear','location','northeast')


% For state x3
subplot(3,2,5)
plot(Lin.t,r2d(2)*Lin.x(:,3),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,3),'k--',...
    NL.t,r2d(2)*NL.x(:,3),'r',...
    NL.t,r2d(2)*NL.x_hat(:,3),'b--')
grid on
xlabel('Time(s)')
ylabel('Position(m)')
title ('State x3: Position of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')

% For state x4
subplot(3,2,6)
plot(Lin.t,r2d(2)*Lin.x(:,4),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,4),'k--',...
    NL.t,r2d(2)*NL.x(:,4),'r',...
    NL.t,r2d(2)*NL.x_hat(:,4),'b--')
grid on
xlabel('Time(s)')
ylabel('Velocity')
title ('State x4: Velocity of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')

%% Plot 2
r2d = [180/pi,1];
figure('Name',' Uncontrolled vs Controlled response: Linear');
%Control Force 
subplot(3,2,1:2)

plot(NL.t,NL.F,'b', Lin.t,Lin.F,'r--')
grid on
xlabel('Time(s)')
ylabel('Force(N)')
title ('Control Force')
legend('Non-linear','Linear')

% For state x1
subplot(3,2,3)
plot(Lin.t,r2d(1)*Lin.x(:,1),'g-', ...
    Lin.t,r2d(1)*Lin.x_hat(:,1),'k--',...
    NL.t,r2d(1)*NL.x(:,1),'r',...
    NL.t,r2d(1)*NL.x_hat(:,1),'b--')
grid on
xlabel('Time(s)')
ylabel('Angle(deg)')
title ('State x1: Angle of rotating actuator')
legend('x1 Linear','x1 hat Linear','x1 Non-Linear','x1 hat Non-Linear','location','northeast')


% For state x2
subplot(3,2,4)
plot(Lin.t,r2d(1)*Lin.x(:,2),'g',...
    Lin.t,r2d(1)*Lin.x_hat(:,2),'k--',...
    NL.t,r2d(1)*NL.x(:,2),'r',...
    NL.t,r2d(1)*NL.x_hat(:,2),'b--')
grid on
xlabel('Time(s)')
ylabel('Angular Velocity(deg/s)')
title ('State x2: Angular velocity of rotating actuator')
legend('x2 Linear','x2 hat Linear','x2 Non-Linear','x2 hat Non-Linear','location','northeast')


% For state x3
subplot(3,2,5)
plot(Lin.t,r2d(2)*Lin.x(:,3),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,3),'k--',...
    NL.t,r2d(2)*NL.x(:,3),'r',...
    NL.t,r2d(2)*NL.x_hat(:,3),'b--')
grid on
xlabel('Time(s)')
ylabel('Position(m)')
title ('State x3: Position of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')

% For state x4
subplot(3,2,6)
plot(Lin.t,r2d(2)*Lin.x(:,4),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,4),'k--',...
    NL.t,r2d(2)*NL.x(:,4),'r',...
    NL.t,r2d(2)*NL.x_hat(:,4),'b--')
grid on
xlabel('Time(s)')
ylabel('Velocity')
title ('State x4: Velocity of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')


%% Plot 3
r2d = [180/pi,1];
figure('Name','Difficult Starting Conditions: Positive angle');
%Plot 1 Control Force 
subplot(3,2,1:2)

plot(NL.t,NL.F,'b', Lin.t,Lin.F,'r--')
grid on
xlabel('Time(s)')
ylabel('Force(N)')
title ('Control Force')
legend('Non-linear','Linear')

% For state x1
subplot(3,2,3)
plot(Lin.t,r2d(1)*Lin.x(:,1),'g-', ...
    Lin.t,r2d(1)*Lin.x_hat(:,1),'k--',...
    NL.t,r2d(1)*NL.x(:,1),'r',...
    NL.t,r2d(1)*NL.x_hat(:,1),'b--')
grid on
xlabel('Time(s)')
ylabel('Angle(deg)')
title ('State x1: Angle of rotating actuator')
legend('x1 Linear','x1 hat Linear','x1 Non-Linear','x1 hat Non-Linear','location','northeast')


% For state x2
subplot(3,2,4)
plot(Lin.t,r2d(1)*Lin.x(:,2),'g',...
    Lin.t,r2d(1)*Lin.x_hat(:,2),'k--',...
    NL.t,r2d(1)*NL.x(:,2),'r',...
    NL.t,r2d(1)*NL.x_hat(:,2),'b--')
grid on
xlabel('Time(s)')
ylabel('Angular Velocity(deg/s)')
title ('State x2: Angular velocity of rotating actuator')
legend('x2 Linear','x2 hat Linear','x2 Non-Linear','x2 hat Non-Linear','location','northeast')


% For state x3
subplot(3,2,5)
plot(Lin.t,r2d(2)*Lin.x(:,3),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,3),'k--',...
    NL.t,r2d(2)*NL.x(:,3),'r',...
    NL.t,r2d(2)*NL.x_hat(:,3),'b--')
grid on
xlabel('Time(s)')
ylabel('Position(m)')
title ('State x3: Position of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')

% For state x4
subplot(3,2,6)
plot(Lin.t,r2d(2)*Lin.x(:,4),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,4),'k--',...
    NL.t,r2d(2)*NL.x(:,4),'r',...
    NL.t,r2d(2)*NL.x_hat(:,4),'b--')
grid on
xlabel('Time(s)')
ylabel('Velocity')
title ('State x4: Velocity of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')

%% Plot 4
r2d = [180/pi,1];
figure('Name','Difficult Starting Conditions: Negative angle');
%Plot 1 Control Force 
subplot(3,2,1:2)

plot(NL.t,NL.F,'b', Lin.t,Lin.F,'r--')
grid on
xlabel('Time(s)')
ylabel('Force(N)')
title ('Control Force')
legend('Non-linear','Linear')

% For state x1
subplot(3,2,3)
plot(Lin.t,r2d(1)*Lin.x(:,1),'g-', ...
    Lin.t,r2d(1)*Lin.x_hat(:,1),'k--',...
    NL.t,r2d(1)*NL.x(:,1),'r',...
    NL.t,r2d(1)*NL.x_hat(:,1),'b--')
grid on
xlabel('Time(s)')
ylabel('Angle(deg)')
title ('State x1: Angle of rotating actuator')
legend('x1 Linear','x1 hat Linear','x1 Non-Linear','x1 hat Non-Linear','location','northeast')


% For state x2
subplot(3,2,4)
plot(Lin.t,r2d(1)*Lin.x(:,2),'g',...
    Lin.t,r2d(1)*Lin.x_hat(:,2),'k--',...
    NL.t,r2d(1)*NL.x(:,2),'r',...
    NL.t,r2d(1)*NL.x_hat(:,2),'b--')
grid on
xlabel('Time(s)')
ylabel('Angular Velocity(deg/s)')
title ('State x2: Angular velocity of rotating actuator')
legend('x2 Linear','x2 hat Linear','x2 Non-Linear','x2 hat Non-Linear','location','northeast')


% For state x3
subplot(3,2,5)
plot(Lin.t,r2d(2)*Lin.x(:,3),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,3),'k--',...
    NL.t,r2d(2)*NL.x(:,3),'r',...
    NL.t,r2d(2)*NL.x_hat(:,3),'b--')
grid on
xlabel('Time(s)')
ylabel('Position(m)')
title ('State x3: Position of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')

% For state x4
subplot(3,2,6)
plot(Lin.t,r2d(2)*Lin.x(:,4),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,4),'k--',...
    NL.t,r2d(2)*NL.x(:,4),'r',...
    NL.t,r2d(2)*NL.x_hat(:,4),'b--')
grid on
xlabel('Time(s)')
ylabel('Velocity')
title ('State x4: Velocity of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')


%% Plot 5
r2d = [180/pi,1];
figure('Name','Final TORA response');
%Plot 1 Control Force 
subplot(3,2,1:2)

plot(NL.t,NL.F,'b', Lin.t,Lin.F,'r--')
grid on
xlabel('Time(s)')
ylabel('Force(N)')
title ('Control Force')
legend('Non-linear','Linear')

% For state x1
subplot(3,2,3)
plot(Lin.t,r2d(1)*Lin.x(:,1),'g-', ...
    Lin.t,r2d(1)*Lin.x_hat(:,1),'k--',...
    NL.t,r2d(1)*NL.x(:,1),'r',...
    NL.t,r2d(1)*NL.x_hat(:,1),'b--')
grid on
xlabel('Time(s)')
ylabel('Angle(deg)')
title ('State x1: Angle of rotating actuator')
legend('x1 Linear','x1 hat Linear','x1 Non-Linear','x1 hat Non-Linear','location','northeast')


% For state x2
subplot(3,2,4)
plot(Lin.t,r2d(1)*Lin.x(:,2),'g',...
    Lin.t,r2d(1)*Lin.x_hat(:,2),'k--',...
    NL.t,r2d(1)*NL.x(:,2),'r',...
    NL.t,r2d(1)*NL.x_hat(:,2),'b--')
grid on
xlabel('Time(s)')
ylabel('Angular Velocity(deg/s)')
title ('State x2: Angular velocity of rotating actuator')
legend('x2 Linear','x2 hat Linear','x2 Non-Linear','x2 hat Non-Linear','location','northeast')


% For state x3
subplot(3,2,5)
plot(Lin.t,r2d(2)*Lin.x(:,3),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,3),'k--',...
    NL.t,r2d(2)*NL.x(:,3),'r',...
    NL.t,r2d(2)*NL.x_hat(:,3),'b--')
grid on
xlabel('Time(s)')
ylabel('Position(m)')
title ('State x3: Position of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')

% For state x4
subplot(3,2,6)
plot(Lin.t,r2d(2)*Lin.x(:,4),'g',...
    Lin.t,r2d(2)*Lin.x_hat(:,4),'k--',...
    NL.t,r2d(2)*NL.x(:,4),'r',...
    NL.t,r2d(2)*NL.x_hat(:,4),'b--')
grid on
xlabel('Time(s)')
ylabel('Velocity')
title ('State x4: Velocity of Translational oscillator')
legend('x3 Linear','x3 hat Linear','x3 Non-Linear','x3 hat Non-Linear','location','northeast')



%% To animations
%(time vector, x1, x2, equilibrium x1, equilibrium x2)
%TORA_Animation(NonLinear_1.t,NonLinear_1.x(:,1),NonLinear_1.x(:,2),x_bar(1),x_bar(2)) 