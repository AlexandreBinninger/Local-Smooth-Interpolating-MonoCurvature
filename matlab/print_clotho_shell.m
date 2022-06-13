function print_clotho_shell(x_point, y_point, theta_s, K, nb_points)
%PRINT_CLOTHO_SHELL
%   Print the clothoid shell and the clothoid

% minimum and maximum angle shift
alpha_min = 0;
alpha_max = 2*pi;

% print the clothoid corresponding to the end with this frequency
frequency_clothoid = 16; 
clotho_shell_endpoint = zeros(nb_points, 2);

for i=0:nb_points
    alpha = alpha_min + i*(alpha_max-alpha_min)/nb_points;
    [x, y] = clotho_shell(alpha, theta_s);
    clotho_shell_endpoint(i+1, 1) = x_point + x / K;
    clotho_shell_endpoint(i+1, 2) = y_point + y / K;
    
    if (mod(i, frequency_clothoid) == 0)
        L = 2*alpha/K;
        points_clotho = zeros(100, 2);
        for j = 0:100
            t = j * L / 100;
            [x, y] = clothoid(alpha, K, theta_s, t);
            points_clotho(j +1, 1) = x_point + x;
            points_clotho(j +1, 2) = y_point + y;
        end
        plot(points_clotho(:, 1), points_clotho(:, 2));
    end
end

plot(clotho_shell_endpoint(:, 1), clotho_shell_endpoint(:, 2), "k-", 'LineWidth', 2);
end

