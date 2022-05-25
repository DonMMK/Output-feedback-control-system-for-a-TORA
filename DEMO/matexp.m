function G = matexp(A,T,n)
% matexp(A,T,n) Calculates a discrete state matrix given
% A = Continuous Time State Matrix
% T = Sample Time for the discretisation
% n = order of approximation for the matrix exponential

G = zeros(size(A));
G_update = eye(size(A));
for k=1:n+1
   G = G + G_update;
   G_update = A*T*G_update/k;
end
