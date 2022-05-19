%% EGH445 Controllability Check

Cab = ctrb(A, B);

if (rank(Cab) == length(A))
    
    fprintf('Equilibrium Point "%s" is Completely Controllable \n\n', EP);
    
else
    
    fprintf('Equilibrium Point "%s" is NOT Completely Controllable \n\n', EP);
    
end
