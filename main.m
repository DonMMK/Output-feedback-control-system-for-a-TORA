%% Parameters

M = 1.3608;    % kg
m = 0.096 ;    % kg
L = 1     ;    % m
J = 0.0002175; % kg m2
k = 186.3;     % N/m


%% Stability

% When the equilibrium point is at 0 - Point A
Aa = [0 , 1 , 0 ,0 ; 
   0 , 0 , (m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) ) , 0;
   0 ,0 ,0 ,1;
   0 , 0, (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  ) , 0;]

EigValues_A = eig(Aa)

sum(EigValues_A > 0)


% When the equilibrium point is at 180 - Point B
Ab = [0 , 1 , 0, 0;
    0 ,0 , (-m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) ) , 0;
    0 ,0 ,0 , 1;
    0 , 0, (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  ) , 0;]

EigValues_B = eig(Ab)

sum(EigValues_B > 0)



%% Control design using state feedback

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



% Controllability Matrix

C_Aa_Ba = [ Ba , Aa * Ba ,  (Aa^2) * Ba ,  (Aa^3) * Ba] ;
Controllablity_Point_a = ctrb(Aa , Ba)

C_Ab_Bb = [ Bb , Ab * Bb ,  (Ab^2) * Bb ,  (Ab^3) * Bb] ;
Controllablity_Point_b = ctrb(Ab , Bb)

% Calculate the rank of the system to check if system is controllable
Rank_at_Point_a = rank(Controllablity_Point_a)
Rank_at_Point_b = rank(Controllablity_Point_b)

if (Rank_at_Point_a & Rank_at_Point_b == 4 ) 
    fprintf("Both eq points are full rank, therefore controllable")
else
    fprintf("Not full rank. The rank is: %d %d " ,Rank_at_Point_a , Rank_at_Point_b )
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




