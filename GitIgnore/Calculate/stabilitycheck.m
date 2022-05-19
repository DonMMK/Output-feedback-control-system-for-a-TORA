%% EGH445 Stability Check

% lyapunov's "indirect" method criterion for non-linear systems'
% stability checking
lambda = eig(A);

if (sum(lambda > 0) >= 1)
    
    fprintf('Equilibrium Point "%s" is Unstable \n\n', EP);
    
elseif (sum(lambda == 0) >= 1)
    
    fprintf('Equilibrium Point "%s" is Lyapunov''s "Indirect" Method Inconclusive \n\n', EP);
    
else
    
    fprintf('Equilibrium Point "%s" is Stable \n\n', EP);
    
end
