function H = intmatexpB(A,B,T,n)
% intmatexpB(A,B,T,n) Calculates a discrete input matrix given
% A = Continuous Time State Matrix
% B = Continuous Time Input Matrix
% T = Sample Time for the discretisation
% n = order of approximation for the matrix exponential

H = zeros(size(A));
H_update = eye(size(A))*T;
for k=2:n+1
   H = H + H_update;
   H_update = A*T*H_update/k;
end
H = H*B;
