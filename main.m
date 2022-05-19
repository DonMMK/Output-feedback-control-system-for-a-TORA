%% EGH445 Modern Control TORA System %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters

M = 1.3608;    % kg Mass of Translating Oscilattor
m = 0.096 ;    % kg Mass of Rotating Actuator
L = 1     ;    % m  Length of Rotating Actuator
J = 0.0002175; % kg m2 Intertia
k = 186.3;     % N/m Spring Constant


%% Stability

% When the equilibrium point is at 0 - Point A
Aa = [0 , 1 , 0 ,0 ; 
   0 , 0 , (m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) ) , 0;
   0 ,0 ,0 ,1;
   0 , 0, (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  ) , 0;]

EigValues_A = eig(Aa)
Stability_A = sum(EigValues_A > 0)
if (Stability_A < 0)
    fprintf("This Equilibrium Point with eig value %d is Stable" , Stability_A)
elseif (Stability_A == 0)
   fprintf("This Equilibrium Point with eig value %d is Inconclusive" , Stability_A) 
else
   fprintf("This Equilibrium Point with eig value %d is Unstable" , Stability_A)
end

% When the equilibrium point is at 180 - Point B
Ab = [0 , 1 , 0, 0;
    0 ,0 , (-m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) ) , 0;
    0 ,0 ,0 , 1;
    0 , 0, (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  ) , 0;]

EigValues_B = eig(Ab)
Stability_B = sum(EigValues_B > 0)
if (Stability_B < 0)
    fprintf("This Equilibrium Point with eig value %d is Stable" , Stability_B)
elseif (Stability_B == 0)
   fprintf("This Equilibrium Point with eig value %d is Inconclusive" , Stability_B) 
else
   fprintf("This Equilibrium Point with eig value %d is Unstable" , Stability_B)
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

% The D matrix is zero for this system
           
           
%% Control design using state feedback
% Controllability Matrix

C_Aa_Ba = [ Ba , Aa * Ba ,  (Aa^2) * Ba ,  (Aa^3) * Ba] ;
Controllablity_Point_a = ctrb(Aa , Ba)

C_Ab_Bb = [ Bb , Ab * Bb ,  (Ab^2) * Bb ,  (Ab^3) * Bb] ;
Controllablity_Point_b = ctrb(Ab , Bb)

% Calculate the rank of the system to check if system is controllable
Rank_at_Point_a = rank(Controllablity_Point_a)
Rank_at_Point_b = rank(Controllablity_Point_b)

if (Rank_at_Point_a == 4 ) 
    fprintf("Both eq points are full rank, therefore controllable")
else
    fprintf("Not full rank. The rank is: %d " ,Rank_at_Point_a )
end

if (Rank_at_Point_b == 4 ) 
    fprintf("Both eq points are full rank, therefore controllable")
else
    fprintf("Not full rank. The rank is: %d " ,Rank_at_Point_b )
end


%% Pole Placement

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
% 2 fast 2 slow???


%Pole 1 and 2 
Poles_1_2 = roots([1, 2*zeta*wn, wn^2]) 
%Pole 3 and 4 
% The rest of the poles ?? 




%% Observability




