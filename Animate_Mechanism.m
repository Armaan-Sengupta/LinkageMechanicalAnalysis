clear; clc; close all;

%% -- Known --
R1 = 8.4;   % Slider track height for D
R2 = 36;    % Length of link AB
R3 = 120;   % Length of slider link BD
R6 = 60;    % Length of coupler AC
theta2_velocity = 2;  

%% -- Kinematic function handles --
f_theta3 = @(theta2) asind((R1 - R2.*sind(theta2))/R3);
f_r4 = @(theta2) R3.*cosd(f_theta3(theta2)) + R2.*cosd(theta2);
f_theta3dot = @(theta2) -1*R2*theta2_velocity*cosd(theta2)./(R3*cosd(f_theta3(theta2)));
f_r4dot = @(theta2) -1*R3.*f_theta3dot(theta2).*sind(f_theta3(theta2)) - R2*theta2_velocity.*sind(theta2);
f_theta3dotdot = @(theta2) (R3.*(f_theta3dot(theta2).^2).*sind(f_theta3(theta2)) + R2*theta2_velocity.*sind(theta2)) ./ (R3.*cosd(f_theta3(theta2)));
f_r4dotdot = @(theta2) -1*R3.*(f_theta3dotdot(theta2).*sind(f_theta3(theta2)) + R3.*f_theta3dot(theta2).^2.*cosd(f_theta3(theta2))) + R2.*theta2_velocity.^2.*cosd(theta2);

f_theta6 = @(theta2) -1*asind(R2/R6*sind(f_theta3(theta2) - theta2)) - f_theta3(theta2);
f_3A = @(theta2) (R6.*cosd(f_theta6(theta2)) - R2.*cosd(theta2))./cosd(f_theta3(theta2));
f_3Adot = @(theta2) (f_3A(theta2).*f_theta3dot(theta2).*(sind(f_theta3(theta2)).*cosd(f_theta6(theta2)) - cosd(f_theta3(theta2)).*sind(f_theta6(theta2))) ...
            + R2*theta2_velocity.*(sind(theta2).*cosd(f_theta6(theta2)) - cosd(theta2).*sind(f_theta6(theta2)))) ...
            ./ (cosd(f_theta3(theta2)).*cosd(f_theta6(theta2)) + sind(f_theta3(theta2)).*sind(f_theta6(theta2)));

f_theta6dot = @(theta2) (f_3Adot(theta2).*cosd(f_theta3(theta2)) - f_3A(theta2).*f_theta3dot(theta2).*sind(f_theta3(theta2)) - R2*theta2_velocity.*sind(theta2)) ./ (-R6.*sind(f_theta6(theta2)));

f_ky = @(theta2) (-1*R6.*(f_theta6dot(theta2).^2).*sind(f_theta6(theta2)) - 2.*f_3Adot(theta2).*f_theta3dot(theta2).*cosd(f_theta3(theta2)) - f_3A(theta2).*f_theta3dotdot(theta2).*cosd(f_theta3(theta2)) + f_3A(theta2).*(f_theta3dot(theta2).^2).*sind(f_theta3(theta2)) + R2.*(theta2_velocity^2).*sind(theta2));
f_kx = @(theta2) -1*R6.*(f_theta6dot(theta2).^2).*cosd(f_theta6(theta2)) + 2.*f_3Adot(theta2).*f_theta3dot(theta2).*sind(f_theta3(theta2)) + f_3A(theta2).*f_theta3dotdot(theta2).*sind(f_theta3(theta2)) + f_3A(theta2).*(f_theta3dot(theta2).^2).*cosd(f_theta3(theta2)) + R2.*(theta2_velocity^2).*cosd(theta2);

f_3Adotdot = @(theta2) -1.*(f_3A(theta2).*sind(f_theta3(theta2)) - f_ky(theta2)).*sind(f_theta6(theta2)./(cosd(f_theta6(theta2)).*cosd(f_theta3(theta2))) + f_kx(theta2)./cosd(f_theta3(theta2)));
f_theta6dotdot = @(theta2) (f_3Adotdot(theta2).*sind(f_theta3(theta2)) - f_ky(theta2)) ./ (R6.*cosd(f_theta6(theta2)));

%% -- Set up figure --
figure('Name','3-Link Mechanism','NumberTitle','off','Position',[200 200 1000 600]);
movegui(gcf, 'center');  % Center the figure on the screen

hold on; grid on; axis equal;
xlabel('X (cm)'); ylabel('Y (cm)');
title('Animation: 3 Links (AB, BD, AC)');
axis([-50 150 -10 50]);

% Ground pivot A at (0,0)
A = [0, 0];
plot(A(1), A(2), 'ks','MarkerSize',8,'MarkerFaceColor','k');

%% -- Range of input angles --
theta2_range = linspace(0, 720, 180);  % 0 to 720 degrees

%% -- Animation Loop --
for k = 1:length(theta2_range)
    t2 = theta2_range(k);
    
    % 1) Point B on link AB (crank rotating about A)
    B = [R2*cosd(t2), R2*sind(t2)];
    
    % 2) Point D on the horizontal slider (y = R1)
    D = [f_r4(t2), R1];
    
    % 3) Determine point C on BD using f_3A equation.
    %    f_3A(t2) returns the distance from B to C along BD.
    BDvec = D - B;       % Vector from B to D
    frac = f_3A(t2) / R3;  % Fraction along BD (R3 is the full length)
    C = B + frac * BDvec;
    
    % 4) Plot the three links:
    %    - Link AB (from A to B)
    %    - Link BD (from B to D)
    %    - Link AC (from A to C)
    hAB = plot([A(1), B(1)], [A(2), B(2)], 'ro-', 'LineWidth', 2);
    hBD = plot([B(1), D(1)], [B(2), D(2)], 'ko-', 'LineWidth', 2);
    hAC = plot([A(1), C(1)], [A(2), C(2)], 'co-', 'LineWidth', 2);
    
    % Mark the points B, C, D
    pB = plot(B(1), B(2), 'bo', 'MarkerFaceColor', 'b');
    pC = plot(C(1), C(2), 'gs', 'MarkerFaceColor', 'g');
    pD = plot(D(1), D(2), 'kd', 'MarkerFaceColor', 'k');
    
    pause(0.05);
    
    delete(hAB); delete(hBD); delete(hAC);
    delete(pB); delete(pC); delete(pD);
end

hold off;
