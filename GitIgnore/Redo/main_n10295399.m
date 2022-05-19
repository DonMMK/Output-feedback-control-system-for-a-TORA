
%% NOTE
%The order of the figures are similar to the order in the report
% Refer Figure 3 for clear illustration of the system behavior

%% Clear workspace and close all Windows  
clc
clear all 
close all 


%% Model Parameters:
M = 1.3608; %Platform Mass
m = 0.096;  %Proof Mass
J = 0.0002175; %Inertia
k = 186.3; %Spring Constant
L= 1; %Distance of Mass From Rotating Axis


%% Linear Simulation 
% Only Equilibium point A is considered as Equilibrium point B was deemed
% unstable during hand calculations of the controlability matrix

% Equilibrium A
A = [0 1 0 0;
    0 0 ((m*L*k)/((J+(m*L^2))*(m+M)-((m^2)*(L^2)))) 0;
    0 0 0 1; 
    0 0 ((-k*(J+(m*L^2)))/((J+(m*L^2))*(m+M)-(m^2*L^2))) 0];

B = [0;
    (m+M/((J+(m*L^2))*(m+M)-(m^2*L^2)));
    0;
    ((-m*L)/((J+m*L^2)*(m+M)-(m^2*L^2)))];

C = [0 0 0 0;
      0 1 0 0;
      0 0 1 0; 
      0 0 0 1 ];
D = zeros(4,1);

%Equlibrium Point for the system
x_bar = [0 0 0 0]';

%Starting Position for the system 
x0 = [10*pi/180 0 0.1 0]';

%% Controller Design 
% Check Controllability of the system 
contA = ctrb(A,B);

rank_contA = rank(contA);
if rank_contA ~= length(A)
    error('T equilibrium point A is not controllable. Rank of the Controllability matrix is %d',rank_contA);
else
    fprintf('Equilibrium Point A is Controllable\n')
    
end    
    

% Pole Placement is carried out using time-domain specification
%Enter required percentage overshoot
P =2;
%Enter required settling time
T = 4;

%  Find pole positions
FindPoles_n10295399

%Stable Poles for Pole placement
p = [r(1) r(2) p3 p4]; 

%Gain matrix
K = place(A,B,p);

%New C and D matrix 
C = [1 0 0 0;
      0 0 1 0];
D = zeros(2,1);

%% Observer Design

G = [x_bar(1); x_bar(3)];

%Check Observability 
obsvA = obsv(A,C);
rank_obsvA = rank(obsvA);
if rank_contA ~= length(A)
    error('The equilibrium point A is not Observable. Rank of the Observability matrix is %d',rank_contA);
else
    fprintf('Equilibrium Point A is Observable\n')
end

%Observer Poles 
desiredPoles = [-63 -64 -165 -166];
L_con = place(A', C', desiredPoles)';


%% Run simulations
%%%% Simulation Parameters
stoptime = 10;
controller_enabled = 1;     
use_state_estimates = 1; 
Disturbance_enabled = 0;
%ODE solver settings:
h = 0.01;

%Run simulation
Lin = sim('Linear_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');
NL = sim('NonLin_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');


%% Create Uncontrolled Plots 
controller_enabled = 0; 
Lin_NC = sim('Linear_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');
NL_NC = sim('NonLin_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');



%% Create simulations for extreme starting positions (Negative) 
controller_enabled = 1; 
x0 = [10*pi/180 0 -0.3 0]';
NL_neg3min = sim('NonLin_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');

x0 = [10*pi/180 0 -0.31 0]';
NL_neg3max = sim('NonLin_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');


%% Create simulations for extreme starting positions (Positive)

x0 = [10*pi/180 0 0.27 0]';
NL_pos3min = sim('NonLin_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');

x0 = [10*pi/180 0 0.28 0]';
NL_pos3max = sim('NonLin_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');


%% Create simulations to simulate a disturbance at 10s

x0 = [10*pi/180 0 0.1 0]';
Disturbance_enabled = 1;
stoptime = 50;
Lin_dis = sim('Linear_n10295399','Solver','ode4','FixedStep','h','StopTime','stoptime');

%% Plot Results:
%The order of the figures are similar to the order in the report
Plots_n10295399

