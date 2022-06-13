%==========================================%
%  Program to display the clothoid shell   %
%==========================================%


% Figure set up
h1 = figure(1);
ax = gca;
hold on;
axis equal;
set(gca,'visible','off')

  
% Parameters     
x0 = 0.0;
y0 = 0.0;
theta_0 = 0;
K = 1.0;
nb_points = 800;

print_clotho_shell(x0, y0, theta_0, K, nb_points);
plot(x0, y0, '+','MarkerSize',15, 'Color',[0 0 0]);

%% Additional shells for tests
%{
x0 = ...;
y0 = ...;
theta_0 = pi + ...;
K = ...;
nb_points = 800;
print_clotho_shell(x0, y0, theta_0, K, nb_points);
plot(x0, y0, '+','MarkerSize',15, 'Color',[0 0 0]);
%}


