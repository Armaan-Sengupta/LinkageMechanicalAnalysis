
%Code for Angles
theta2 = linspace(0, 400, 100);
R1 = 8.4;  % Example known value
R2 = 36;  % Example known value
R3 = 120;
R6 = 60;
theta2_velocity = 2;

f_theta3 = @(theta2) asind((R1 - R2.*sind(theta2))/R3);
f_r4 = @(theta2) R3.*cosd(f_theta3(theta2)) + R2.*cosd(theta2);
f_theta3dot = @(theta2) -1*R2*theta2_velocity*cosd(theta2)./(R3*cosd(f_theta3(theta2)));
f_r4dot = @(theta2) -1*R3.*f_theta3dot(theta2).*sind(f_theta3(theta2)) - R2*theta2_velocity.*sind(theta2);
f_theta3dotdot = @(theta2) (R3.*(f_theta3dot(theta2).^2).*sind(f_theta3(theta2)) + R2*theta2_velocity.*sind(theta2)) ./ (R3.*cosd(f_theta3(theta2)));
f_r4dotdot = @(theta2) -1*R3.*(f_theta3dotdot(theta2).*sind(f_theta3(theta2)) + R3.*f_theta3dot(theta2).^2.*cosd(f_theta3(theta2))) + R2.*theta2_velocity.^2.*cos(f_theta3(theta2));


f_theta6 = @(theta2) -1*asind(R2/R6*sind(f_theta3(theta2) - theta2)) - f_theta3(theta2);
f_3A = @(theta2) (R6.*cosd(f_theta6(theta2)) - R2.*cosd(theta2))./cosd(f_theta3(theta2));
f_3Adot = @(theta2) (f_3A(theta2).*f_theta3dot(theta2).*(sind(f_theta3(theta2)).*cosd(f_theta6(theta2)) - cosd(f_theta3(theta2)).*sind(f_theta6(theta2))) ...
            + R2*theta2_velocity.*(sind(theta2).*cosd(f_theta6(theta2)) - cosd(theta2).*sind(f_theta6(theta2)))) ...
            ./ (cosd(f_theta3(theta2)).*cosd(f_theta6(theta2)) + sind(f_theta3(theta2)).*sind(f_theta6(theta2)));

f_theta6dot = @(theta2) (f_3Adot(theta2).*cosd(f_theta3dot(theta2)) - f_3A(theta2).*f_theta3dot(theta2).*sind(f_theta3dot(theta2)) - R2*theta2_velocity.*sind(theta2)) ./ -1*R6.*sind(f_theta6(theta2));

f_ky = @(theta2) (-1.*R6.*(f_theta6dot(theta2).^2).*sind(f_theta6(theta2)) - 2.*f_3Adot(theta2).*f_theta3dot(theta2).*cosd(f_theta3(theta2)) - f_3A(theta2).*f_theta3dotdot(theta2).*cosd(f_theta3(theta2)) + f_3A.*(f_theta3dot(theta2).^2).*sind(f_theta3(theta2)) + R2*(theta2_velocity^2).*sin(theta2));
f_kx = @(theta2) -1*R6.*(f_theta6dot(theta2).^2).*cosd(f_theta6(theta2)) + 2.*f_3Adot(theta2).*f_theta3(theta2).*sind(f_theta3dot(theta2)) + f_3A(theta2).*f_theta3dotdot(theta2).*sind(f_theta3(theta2));

% Create a 2x2 grid of subplots
figure;

% Plot 1: f_theta3
subplot(3, 4, 1);  % 2x2 grid, plot 1
plot(theta2, f_theta3(theta2), 'LineWidth', 2);
title('f\_theta3');
xlabel('theta2');
ylabel('f\_theta3');
grid on;

% Plot 2: f_r4
subplot(3, 4, 2);  % 2x2 grid, plot 2
plot(theta2, f_r4(theta2), 'LineWidth', 2);
title('f\_r4');
xlabel('theta2');
ylabel('f\_r4');
grid on;

% Plot 3: theta3dot
subplot(3, 4, 3);  % 2x2 grid, plot 3
plot(theta2, f_theta3dot(theta2), 'LineWidth', 2);
title('f\_theta3dot');
xlabel('theta2');
ylabel('f\_ky');
grid on;

% Plot 4: r4_dot
subplot(3, 4, 4);  % 2x2 grid, plot 4
plot(theta2, f_r4dot(theta2), 'LineWidth', 2);
title('f\_r4dot');
xlabel('theta2');
ylabel('f\_kx');
grid on;

subplot(3, 4, 5);  % 2x2 grid, plot 4
plot(theta2, f_r4dotdot(theta2), 'LineWidth', 2);
title('f\_r4dotdot');
xlabel('theta2');
ylabel('f\_kx');
grid on;

subplot(3, 4, 6);  % 2x2 grid, plot 4
plot(theta2, f_theta3dotdot(theta2), 'LineWidth', 2);
title('f\_theta3dotdot');
xlabel('theta2');
ylabel('f\_kx');
grid on;

subplot(3, 4, 7);  % 2x2 grid, plot 4
plot(theta2, f_theta6(theta2), 'LineWidth', 2);
title('f\_theta6');
xlabel('theta2');
ylabel('f\_kx');
grid on;

subplot(3, 4, 8);  % 2x2 grid, plot 4
plot(theta2, f_3A(theta2), 'LineWidth', 2);
title('f\_3A');
xlabel('theta2');
ylabel('f\_kx');
grid on;

subplot(3, 4, 9);  % 2x2 grid, plot 4
plot(theta2, f_theta6dot(theta2), 'LineWidth', 2);
title('f\_theta6dot');
xlabel('theta2');
ylabel('f\_kx');
grid on;

subplot(3, 4, 10);  % 2x2 grid, plot 4
plot(theta2, f_3Adot(theta2), 'LineWidth', 2);
title('f\_3Adot');
xlabel('theta2');
ylabel('f\_kx');
grid on;