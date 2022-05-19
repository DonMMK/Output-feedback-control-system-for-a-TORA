%% EGH445 Normal Controller

% damping ratio, natural frequency & roots
zeta = -log(PO/100)/sqrt(pi^2 + log(PO/100)^2);
wn = 4 / (Ts * zeta);

% 2nd order TF form
r = roots([1, 2*zeta*wn, wn^2]);

% 2 slow poles and 2 fast poles
lambda1 = r(1); 
lambda2 = r(2);
lambda3 = ndpl1 * lambda1;
lambda4 = ndpl2 * lambda2;

% controller array
controlDE = [lambda1 lambda2 lambda3 lambda4];

% 'acker' vs 'place' function logic
if numel(unique(real(controlDE))) == 4
    Knormal = place(A, B, controlDE);
else
    Knormal = acker(A, B, controlDE);
end
