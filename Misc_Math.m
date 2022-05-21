syms x1 x2 x3 x4

numerator_2   =  (m+M)*u - m*L*cos(x1)*(m*L*(x2^2)*sin(x1) - k*x3) ;
denominator_2 =  (J+m*L^2)*(m+M) - (m^2)*(L^2)*L*cos(x1)*cos(x1);

numerator_4   =  -m*L*u*cos(x1) + (J+m*(L^2)) *((m*L*(x2^2)*sin(x1)) - k*x3);
denominator_4 =  (J+m*L^2)*(m+M) - (m^2)*(L^2)*L*cos(x1)*cos(x1);

state_1 = x2 ;
state_2 = numerator_2 / denominator_2 ;
state_3 = x4 ;
state_4 = numerator_4 / denominator_4 ;

jacobian([state_1 , state_2 , state_3 , state_4 ],[x1 , x2 , x3, x4])