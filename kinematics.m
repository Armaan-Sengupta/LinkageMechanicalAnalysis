
% Declare constants for link lengths, range for theta2, and theta2 angular velocity
theta2 = linspace(0, 400, 100);
R1 = 8.4;  % Example known value
R2 = 36;  % Example known value
R3 = 120;
R6 = 60;
theta2_velocity = 2;

% Function for theta3 with respect to theta2
f_theta3 = @(theta2) 180 - asind((R1 - R2.*sind(theta2))/R3);

% Function for length of link 4 with respect to theta2
f_r4 = @(theta2) R3.*cosd(f_theta3(theta2)) + R2.*cosd(theta2);

% Function for theta3 velocity with respect to theta2
f_theta3dot = @(theta2) R2*theta2_velocity*cosd(theta2)./(R3*cosd(f_theta3(theta2)));

% Function for length of link 4 velocity with respect to theta2
f_r4dot = @(theta2) -1*R3.*f_theta3dot(theta2).*sind(f_theta3(theta2)) - R2*theta2_velocity.*sind(theta2);

% Function for theta3 acceleration with respect to theta2
f_theta3dotdot = @(theta2) -1*(R3.*(f_theta3dot(theta2).^2).*sind(f_theta3(theta2)) + R2*theta2_velocity.*sind(theta2)) ./ (R3.*cosd(f_theta3(theta2)));

% Function for length of link 4 acceleration with respect to theta2
f_r4dotdot = @(theta2) -1*R3.*(f_theta3dotdot(theta2).*sind(f_theta3(theta2)) + R3.*(f_theta3(theta2).^2).*cosd(f_theta3(theta2))) + R2.*theta2_velocity.^2.*cosd(theta2);

% Function for theta6 with respect to theta2
f_theta6 = @(theta2) -1*asind(R2/R6*sind(f_theta3(theta2) - theta2)) - f_theta3(theta2);

% Function for length of link 3A with respect to theta2
f_3A = @(theta2) (R6.*cosd(f_theta6(theta2)) - R2.*cosd(theta2))./cosd(f_theta3(theta2));

% Function for velocity of length of link 3A with respect to theta2
f_3Adot = @(theta2) (f_3A(theta2).*f_theta3dot(theta2).*(sind(f_theta3(theta2)).*cosd(f_theta6(theta2)) - cosd(f_theta3(theta2)).*sind(f_theta6(theta2))) ...
            + R2*theta2_velocity.*(sind(theta2).*cosd(f_theta6(theta2)) - cosd(theta2).*sind(f_theta6(theta2)))) ...
            ./ (cosd(f_theta3(theta2)).*cosd(f_theta6(theta2)) + sind(f_theta3(theta2)).*sind(f_theta6(theta2)));

% Function for velocity of theta6 with respect to theta2
f_theta6dot = @(theta2) (f_3Adot(theta2).*cosd(f_theta3(theta2)) - f_3A(theta2).*f_theta3dot(theta2).*sind(f_theta3(theta2)) - R2*theta2_velocity.*sind(theta2)) ./ (-R6.*sind(f_theta6(theta2)));

% Helper function 1
f_ky = @(theta2) (-1*R6.*(f_theta6dot(theta2).^2).*sind(f_theta6(theta2)) - 2.*f_3Adot(theta2).*f_theta3dot(theta2).*cosd(f_theta3(theta2)) - f_3A(theta2).*f_theta3dotdot(theta2).*cosd(f_theta3(theta2)) + f_3A(theta2).*(f_theta3dot(theta2).^2).*sind(f_theta3(theta2)) + R2.*(theta2_velocity^2).*sind(theta2));

% Helper function 2
f_kx = @(theta2) -1*R6.*(f_theta6dot(theta2).^2).*cosd(f_theta6(theta2)) + 2.*f_3Adot(theta2).*f_theta3dot(theta2).*sind(f_theta3(theta2)) + f_3A(theta2).*f_theta3dotdot(theta2).*sind(f_theta3(theta2)) + f_3A(theta2).*(f_theta3dot(theta2).^2).*cosd(f_theta3(theta2)) + R2.*(theta2_velocity^2).*cosd(theta2);

% Function for acceleration of link 3A with respect to theta2
f_3Adotdot = @(theta2) -1.*(f_3A(theta2).*sind(f_theta3(theta2)) - f_ky(theta2)).*sind(f_theta6(theta2)./(cosd(f_theta6(theta2)).*cosd(f_theta3(theta2))) + f_kx(theta2)./cosd(f_theta3(theta2)));

% Function for acceleration of theta6 with respect to theta2
f_theta6dotdot = @(theta2) (f_3Adotdot(theta2).*sind(f_theta3(theta2)) - f_ky(theta2)) ./ (R6.*cosd(f_theta6(theta2)));


% Create a 2x2 grid of subplots
figure;

% Plot 1: f_theta3
subplot(3, 4, 1);
plot(theta2, f_theta3(theta2), 'LineWidth', 2);
title('$\theta_3$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\theta_3$', 'Interpreter', 'latex');
grid on;

% Plot 2: f_r4
subplot(3, 4, 2);
plot(theta2, f_r4(theta2), 'LineWidth', 2);
title('$R_4$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$R_4$', 'Interpreter', 'latex');
grid on;

% Plot 3: theta3dot
subplot(3, 4, 3);
plot(theta2, f_theta3dot(theta2), 'LineWidth', 2);
title('$\dot{\theta}_3$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\dot{\theta}_3$', 'Interpreter', 'latex');
grid on;

% Plot 4: r4_dot
subplot(3, 4, 4);
plot(theta2, f_r4dot(theta2), 'LineWidth', 2);
title('$\dot{R}_4$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\dot{R}_4$', 'Interpreter', 'latex');
grid on;

% Plot 5: r4_dotdot
subplot(3, 4, 5);
plot(theta2, f_r4dotdot(theta2), 'LineWidth', 2);
title('$\ddot{R}_4$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\ddot{R}_4$', 'Interpreter', 'latex');
grid on;

% Plot 6: theta3dotdot
subplot(3, 4, 6);
plot(theta2, f_theta3dotdot(theta2), 'LineWidth', 2);
title('$\ddot{\theta}_3$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\ddot{\theta}_3$', 'Interpreter', 'latex');
grid on;

% Plot 7: theta6
subplot(3, 4, 7);
plot(theta2, f_theta6(theta2), 'LineWidth', 2);
title('$\theta_6$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\theta_6$', 'Interpreter', 'latex');
grid on;

% Plot 8: f_3A
subplot(3, 4, 8);
plot(theta2, f_3A(theta2), 'LineWidth', 2);
title('$3A$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$3A$', 'Interpreter', 'latex');
grid on;

% Plot 9: theta6dot
subplot(3, 4, 9);
plot(theta2, f_theta6dot(theta2), 'LineWidth', 2);
title('$\dot{\theta}_6$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\dot{\theta}_6$', 'Interpreter', 'latex');
grid on;

% Plot 10: f_3Adot
subplot(3, 4, 10);
plot(theta2, f_3Adot(theta2), 'LineWidth', 2);
title('$\dot{3A}$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\dot{3A}$', 'Interpreter', 'latex');
grid on;

% Plot 11: f_3Adotdot
subplot(3, 4, 11);
plot(theta2, f_3Adotdot(theta2), 'LineWidth', 2);
title('$\ddot{3A}$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\ddot{3A}$', 'Interpreter', 'latex');
grid on;

% Plot 12: theta6dotdot
subplot(3, 4, 12);
plot(theta2, f_theta6dotdot(theta2), 'LineWidth', 2);
title('$\ddot{\theta}_6$ vs $\theta_2$', 'Interpreter', 'latex');
xlabel('$\theta_2$', 'Interpreter', 'latex');
ylabel('$\ddot{\theta}_6$', 'Interpreter', 'latex');
grid on;
