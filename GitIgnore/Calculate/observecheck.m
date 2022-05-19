%% EGH445 Observability Check

Oac = obsv(A, Cnew);

if (rank(Oac) == length(A))
    
    fprintf('Equilibrium Point "%s" is Completely Observable \n\n', EP);
    
else
    
    fprintf('Equilibrium Point "%s" is NOT Completely Observable \n\n', EP);
    
end
