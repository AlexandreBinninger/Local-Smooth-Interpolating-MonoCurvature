function [x, y] = clotho_shell(alpha, theta_s)
%CLOTHO_SHELL 
%   Compute the normalized clothoid shell for theta_s in [0, 2pi]
cos_part = @(u) cos(alpha*(-u.^2 + 2*u) + theta_s);
sin_part = @(u) sin(alpha*(-u.^2 + 2*u) + theta_s);

x = 2*alpha * integral(cos_part, 0, 1);
y = 2*alpha * integral(sin_part, 0, 1);
end

