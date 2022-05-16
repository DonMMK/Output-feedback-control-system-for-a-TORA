%% Parameters

M = 1.3608;    % kg
m = 0.096 ;    % kg
L = 1     ;    % m
J = 0.0002175; % kg m2
k = 186.3;     % N/m

%% Matrix values

alpha_a = (m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) )

beta_a = (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  )

alpha_b = (-m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) )

beta_b =  (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  )


%% Eigenvalues

% When the equilibrium point is at 0
Aa_matrix = [0 , 1 , 0 ,0 ; 
   0 , 0 , alpha_a , 0;
   0 ,0 ,0 ,1;
   0 , 0, beta_a , 0;]

eig(Aa_matrix)


% When the equilibrium point is at 180
Ab_matrix = [0 , 1 , 0, 0;
    0 ,0 , alpha_b , 0;
    0 ,0 ,0 , 1;
    0 , 0, beta_b , 0;]

eig(Ab_matrix)


%% Control design using state feedback

% Equilibrium Point A
Ba_matrix = [0;
               (m+M/((J+(m*L^2))*(m+M)-(m^2*L^2))) ;
               0 ;
               ((-m*L)/((J+m*L^2)*(m+M)-(m^2*L^2))) ];

% Equilibrium Point B
Bb_matrix = [0;
               Bb_2nd_row ;
               0 ;
               Bb_4th_row ];



% Controllability Matrix

C_Aa_Ba = [ Ba_matrix , Aa_matrix * Ba_matrix ,  (Aa_matrix^2) * Ba_matrix ,  (Aa_matrix^3) * Ba_matrix] ;
Controllablity_a = ctrb(Aa_matrix , Ba_matrix)

C_Ab_Bb = [ Bb_matrix , Ab_matrix * Bb_matrix ,  (Ab_matrix^2) * Bb_matrix ,  (Ab_matrix^3) * Bb_matrix] ;
Controllablity_b = ctrb(Ab_matrix , Bb_matrix)
