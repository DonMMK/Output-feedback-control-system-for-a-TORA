M = 1.3608;    % kg
m = 0.096 ;    % kg
L = 1     ;    % m
J = 0.0002175; % kg m2
k = 186.3;     % N/m

alpha_a = (m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) )

beta_a = (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  )

alpha_b = (-m * L * k )  / ( (J + m * (L^2) ) *(m+M) - (m^2)*(L^2) )

beta_b =  (-k * (J + m *(L*L)) ) / (  (J + m * (L^2) ) *(m+M) - (m^2)*(L^2)  )


Aa_matrix = [0 , 1 , 0 ,0 ; 
   0 , 0 , alpha_a , 0;
   0 ,0 ,0 ,1;
   0 , 0, beta_a , 0;]

eig(Aa_matrix)


Ab_matrix = [0 , 1 , 0, 0;
    0 ,0 , alpha_b , 0;
    0 ,0 ,0 , 1;
    0 , 0, beta_b , 0;]

eig(Ab_matrix)

