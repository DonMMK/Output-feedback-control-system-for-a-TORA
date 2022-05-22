%% EGH445 Modern Control TORA System %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
   0 , 0 , (m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) ) , 0;
   0 ,0 ,0 ,1;
   0 , 0, (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  ) , 0;]

EigValues_A = eig(Aa)
if (sum(EigValues_A < 0) >= 1)
    fprintf("This Equilibrium Point with eig value %d is Stable\n" , EigValues_A)
elseif (sum(EigValues_A == 0) >= 1)
   fprintf("This Equilibrium Point with eig value %d is Marginally Stable\n" , EigValues_A) 
else
   fprintf("This Equilibrium Point with eig value %d is Unstable\n" , EigValues_A)
end

% ASK FOR CLARIFICATION: is it meant to be inconclusive

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
C = eye(4)

% The D matrix is zero for this system
D = zeros(2,2)

%% Pole Placement

% Required Settling Time
Required_Settlting_Time = 4;  % ASK FOR CLARIFICATION:

% Percentage Overshoot
Percentage_Overshoot = 2; % ASK FOR CLARIFICATION:

% Equilibrium Point for the system
x_bar = [0 , 0 , 0 , 0]';

% Starting position of the system
ctoRadians = pi/180;
x0 = [20*(ctoRadians) , 0 , 0.1 0]';

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

if (Rank_at_Point_a == 4 ) 
    fprintf("At point A matrix is full rank, therefore controllable\n")
else
    fprintf("Not full rank. The rank is: %d " ,Rank_at_Point_a )
end

if (Rank_at_Point_b == 4 ) 
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

desiredPoles = [-60 , -60 , -160 , -160];
L_Value = place(Aa', C', desiredPoles)'; % ASK FOR CLARIFICATION: place vs acker

Observer_In = [x_bar(1); x_bar(3)] % ASK FOR CLARIFICATION:


%% Other parameters to set before simulations

stop_time = 10;
stateEstimates = 1;
Boolean_Flag_for_Controller = 1;

% ODE Solver step size
h = 0.01;

%% Simulation 1: Standard Simulation

NonLinear_1 = sim('TORA_Non_Linear', 'Solver','ode4','FixedStep','h','StopTime','stop_time');
Linear_1 = sim('TORA_Linear','Solver','ode4','FixedStep','h','StopTime','stop_time' );

%% Simulation 2: Simulation without the controller

Boolean_Flag_for_Controller = 0;
NonLinear_2 = sim('TORA_Non_Linear', 'Solver','ode4','FixedStep','h','StopTime','stop_time');
Linear_2 = sim('TORA_Linear','Solver','ode4','FixedStep','h','StopTime','stop_time' );

%% Simulation 3: Starting Conditions varied to test controller

x0 = [10*pi/180 0 -0.3 0]';
Boolean_Flag_for_Controller = 0;
NonLinear_3 = sim('TORA_Non_Linear', 'Solver','ode4','FixedStep','h','StopTime','stop_time');
Linear_3 = sim('TORA_Linear','Solver','ode4','FixedStep','h','StopTime','stop_time' );

%% Plot 1: 

figure('Name','Controlled Vs Uncontrolled (Linear System)');
%Plot 1 Control Force 
plot(Linear_1.t,Linear_1.F,'b',Linear_2.t,Linear_2.F,'r--')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('F Linear Controlled','F Linear Uncontrolled')

%% Plot 2: 

figure('Name','Controlled Vs Extreme (Linear System)');
%Plot 1 Control Force 
plot(Linear_1.t,Linear_1.F,'b',Linear_3.t,Linear_3.F,'r--')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('F Linear Controlled','F Linear extreme')

%% Plot 4: 

figure('Name','Linear Vs Non Linear ');
%Plot 1 Control Force 
plot(Linear_1.t,Linear_1.F,'b',NonLinear_1.t,NonLinear_1.F,'r--')
grid on
xlabel('Seconds')
ylabel('Newton')
title ('Control Force')
legend('Linear','NonLinear ')

%% To animations
%(time vector, x1, x2, equilibrium x1, equilibrium x2)
%TORA_Animation(NonLinear_1.t,NonLinear_1.x(:,1),NonLinear_1.x(:,2),x_bar(1),x_bar(2)) 