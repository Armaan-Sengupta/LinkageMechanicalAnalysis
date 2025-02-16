% Define the function you want to plot

%Code for Angles
theta2 = linspace(0, 400, 100);

f_theta3 = @(theta2) asind((8.4 - 36.*sind(theta2))/120);  % Example function: y = x^2
theta3 = f_theta3(theta2);
f_r4 = @(theta2) 120*cosd(theta3) + 36*cosd(theta2);
r4 = f_r4(theta2);
f_theta3dot = @(theta2) -1*36*114.592*cosd(theta2)./(120*cosd(theta3));
theta3dot = f_theta3dot(theta2);
f_r4dot = @(theta2) -1*120.*theta3dot.*sind(theta3) - 36*114.592.*sind(theta2);
r4dot = f_r4dot(theta2);
f_theta3dotdot = @(theta2) (120.*(theta3dot.^2).*sind(theta3) + 36*114.592.*sind(theta2)) ./ (120.*cosd(theta3));
theta3dotdot = f_theta3dotdot(theta2);
f_r4dotdot = @(theta2) -1*120.*(theta3dotdot.*sind(theta3) + 120.*theta3dot.^2.*cosd(theta3)) + 36.*114.592.^2.*cos(theta3);
r4dotdot = f_r4dotdot(theta2);


% Create a range of values for the variable (e.g., x values)
  % 100 points between -10 and 10

% Calculate the corresponding y values


% Create the plot
plot(theta2, r4dotdot, 'LineWidth', 2);  % Line width for better visibility

% Add title and labels
title('Graph of ');
xlabel('x');
ylabel('y');

% Show the grid
grid on;