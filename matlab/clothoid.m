function [x, y] = clothoid(alpha, K, theta_s, t)
%CLOTHOID
%   Compute the clothoid which form a smooth transition wih the origin,
%   initial angle theta_s and initial curvature K, and the line of
%   angle Phi = alpha + theta_s.

k = K;
L = 2*alpha/k;
if (abs(L) <= 1e-5)
   x = 0;
   y = 0;
else
k_prime = - k / L;

cos_part = @(u) cos((k_prime/2)*u.^2 + k*u + theta_s);
sin_part = @(u) sin((k_prime/2)*u.^2 + k*u + theta_s);


x = integral(cos_part, 0, t);
y = integral(sin_part, 0, t);
end
end

