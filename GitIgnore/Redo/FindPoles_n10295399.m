

zeta = -log(P/100)/sqrt(pi^2+log(P/100)^2);
wn = 4/(T*zeta);

%Pole 1 and 2 
r = roots([1, 2*zeta*wn, wn^2]);
%Pole 3 and 4 
p3 = r(1)*8;
p4 = r(2)*8;


