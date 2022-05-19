% function [x2_dot, x4_dot] = fcn(m, M, L, J, k, T, x1, x2, x3)
%     
%     
%     delTheta = (J + m*L^2)*(m + M) - m^2*L^2.*cos(x1).^2;
% 
%     x2_dot = ((m + M)*T  - m*L.*cos(x1).*(m*L*x2.^2.*sind(x1) - k*x3))./delTheta;
%     x4_dot = (-m*L*T.*cos(x1) + (J + m*L^2).*(m*L*x2.^2.*sind(x1) - k*x3))./delTheta;
% 
% end

function [x2_dot, x4_dot] = fcn(m, M, L, J, k, T, x1, x2, x3)
    
    
    delTheta = (J + m*L^2) * (m + M) - m^2*L^2 .* cos(x1).^2;
    mid = (m*L*x2.^2 .* sin(x1) - k*x3);

    x2_dot = ((m + M)*T  - m*L.*cos(x1) .* mid) ./ delTheta;
    x4_dot = (-m*L*T.*cos(x1) + (J + m*L^2) .* mid) ./ delTheta;

y = u;