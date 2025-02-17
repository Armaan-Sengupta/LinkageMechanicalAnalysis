%% Kinetics

clear; clc; close all;

% Declare constants for link lengths, range for theta2, and theta2 angular velocity
theta2 = linspace(0, 400, 100);
R1 = 8.4;  % Example known value
R2 = 36;  % Example known value
R3 = 120;
R6 = 60;
theta2_velocity = 2;

% Function for theta3 with respect to theta2
f_theta3 = @(theta2) asind((R1 - R2.*sind(theta2))/R3);

% Function for length of link 4 with respect to theta2
f_r4 = @(theta2) R3.*cosd(f_theta3(theta2)) + R2.*cosd(theta2);

% Function for theta3 velocity with respect to theta2
f_theta3dot = @(theta2) R2*theta2_velocity*cosd(theta2)./(R3*cosd(f_theta3(theta2)));

% Function for length of link 4 velocity with respect to theta2
f_r4dot = @(theta2) -1*R3.*f_theta3dot(theta2).*sind(f_theta3(theta2)) - R2*theta2_velocity.*sind(theta2);

% Function for theta3 acceleration with respect to theta2
f_theta3dotdot = @(theta2) -1*(R3.*(f_theta3dot(theta2).^2).*sind(f_theta3(theta2)) + R2*theta2_velocity.*sind(theta2)) ./ (R3.*cosd(f_theta3(theta2)));

% Function for length of link 4 acceleration with respect to theta2
f_r4dotdot = @(theta2) -1*R3.*(f_theta3dotdot(theta2).*sind(f_theta3(theta2)) + (f_theta3dot(theta2).^2).*cosd(f_theta3(theta2))) - (R2.*(theta2_velocity.^2)).*cosd(theta2);

% Function for theta6 with respect to theta2
f_theta6 = @(theta2) -1*asind(R2/R6.*sind(f_theta3(theta2) - theta2)) - f_theta3(theta2);

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
f_3Adotdot = @(theta2) (- (f_3A(theta2).*sind(f_theta3(theta2)) - f_ky(theta2)).*sind(f_theta6(theta2)) + f_kx(theta2)) ./ (cosd(f_theta3(theta2)).*cosd(f_theta6(theta2)));

% Function for acceleration of theta6 with respect to theta2
f_theta6dotdot = @(theta2) (f_3Adotdot(theta2).*sind(f_theta3(theta2)) - f_ky(theta2)) ./ (R6.*cosd(f_theta6(theta2)));

f_corlias_acceleration = @(theta2) -2.*f_theta3dot(theta2).*f_3Adot(theta2);



%% Part 2 - Force and Moment Calculation


%%initial parameters:

dtheta_2 = 2; % theta2 dot
ddtheta_2 = 0; % theta2 doble-dot - second derivative


theta2_list = [];
alpha_s_list = []; % direction of a shaking force
Fij_list = []; % Forces
Fij_alpha = []; % Angles at which forces are acting

theta2_vals = 0:1:360;
numPoints   = length(theta2_vals);
% Preallocate for results
F1_mag   = zeros(numPoints,1);
F2_mag   = zeros(numPoints,1);
F3_mag   = zeros(numPoints,1);
F4_mag   = zeros(numPoints,1);
F5_mag   = zeros(numPoints,1);
F6_mag   = zeros(numPoints,1);
F14_mag  = zeros(numPoints,1);

M12_list = zeros(numPoints,1);
Fs_list  = zeros(numPoints,1);
Ms_list  = zeros(numPoints,1);


for theta2 = 1:numPoints
    theta2deg=theta2_vals(theta2)
    theta2rad=deg2rad(theta2);

    % kinematic variables are caculated based on loop eqn
    r_i = [0,0.36,1.2,0,0,0.6];
    dr_i = [];
    ddr_i = [0,0,0,f_r4dotdot(theta2deg),0,0];
    theta_i = [0,theta2rad,deg2rad(f_theta3(theta2deg)),0,0,deg2rad(f_theta6(theta2deg))];
    dtheta_i = [0,dtheta_2,f_theta3dot(theta2deg),0,0,f_theta6dot(theta2deg)];
    ddtheta_i = [0,0,f_theta3dotdot(theta2deg),0,0,f_theta6dotdot(theta2deg)];


    
    B = get_ma_vector_Skeleton(theta_i, dtheta_i, ddtheta_i, ...
                             r_i, ddr_i);
    r_3A = f_3A(theta2deg)*0.01;
    theta_i = [0,theta2rad,deg2rad(f_theta3(theta2deg)),0,0,deg2rad(f_theta6(theta2deg))];
    A = get_A_matrix_Skeleton(theta_i, r_i(2), r_i(3), r_3A, r_i(6));

    x = A\ B % Ax = B, solution for x

 % Pull out each unknown from x:
    F1x   = x(1);
    F1y   = x(2);
    F2x   = x(3);
    F2y   = x(4);
    F3x   = x(5);
    F3y   = x(6);
    F4x   = x(7);
    F4y   = x(8);
    F5    = x(9);
    F6x   = x(10);
    F6y   = x(11);
    F14y  = x(12);
    M12   = x(13);

    % -----------------------------------------------------------
    % (E)  Magnitudes of each joint force
    % -----------------------------------------------------------
    F1_mag(theta2)  = sqrt(F1x^2 + F1y^2);
    F2_mag(theta2)  = sqrt(F2x^2 + F2y^2);
    F3_mag(theta2)  = sqrt(F3x^2 + F3y^2);
    F4_mag(theta2)  = sqrt(F4x^2 + F4y^2);
    F5_mag(theta2)  = abs(F5);  % scalar, direction known from geometry
    F6_mag(theta2)  = sqrt(F6x^2 + F6y^2);
    F14_mag(theta2) = abs(F14y);  % only a Y‐component

    % -----------------------------------------------------------
    % (F)  Shaking force and moment
    % -----------------------------------------------------------
    % Shaking force:
    Fs_x = F2x + F1x;
    Fs_y = F2y + F1y - F14y;
    Fs   = sqrt(Fs_x^2 + Fs_y^2);

    % Shaking moment:
    R4=f_r4(theta2)*0.01;
    Ms = M12 - (R4*1e-2)*F14y; 

    % Store for plotting
    M12_list(theta2) = M12;
    Fs_list(theta2)  = Fs;
    Ms_list(theta2)  = Ms;

end  % end for loop over theta2

% -------------------------------
% 3) Make the plots
% -------------------------------

figure('Name','Individual Force Magnitudes','NumberTitle','off');

subplot(4,2,1)
plot(theta2_vals, F1_mag, 'r','LineWidth',1.5);
title('F1 Magnitude');
xlabel('\theta_2 [deg]');
ylabel('Force [N]');
grid on;

subplot(4,2,2)
plot(theta2_vals, F2_mag, 'b','LineWidth',1.5);
title('F2 Magnitude');
xlabel('\theta_2 [deg]');
ylabel('Force [N]');
grid on;

subplot(4,2,3)
plot(theta2_vals, F3_mag, 'g','LineWidth',1.5);
title('F3 Magnitude');
xlabel('\theta_2 [deg]');
ylabel('Force [N]');
grid on;

subplot(4,2,4)
plot(theta2_vals, F4_mag, 'm','LineWidth',1.5);
title('F4 Magnitude');
xlabel('\theta_2 [deg]');
ylabel('Force [N]');
grid on;

subplot(4,2,5)
plot(theta2_vals, F5_mag, 'c','LineWidth',1.5);
title('F5 Magnitude');
xlabel('\theta_2 [deg]');
ylabel('Force [N]');
grid on;

subplot(4,2,6)
plot(theta2_vals, F6_mag, 'k','LineWidth',1.5);
title('F6 Magnitude');
xlabel('\theta_2 [deg]');
ylabel('Force [N]');
grid on;

subplot(4,2,7)
plot(theta2_vals, F14_mag, 'y','LineWidth',1.5);
title('F_{14} Magnitude');
xlabel('\theta_2 [deg]');
ylabel('Force [N]');
grid on;

subplot(4,2,8)
plot(theta2_vals, Fs_list, 'LineWidth',1.5);
title('Shaking Force |F_s|');
xlabel('\theta_2 [deg]');
ylabel('Force [N]');
grid on;

figure('Name','Moment Plots','NumberTitle','off');


subplot(2,1,1)
plot(theta2_vals, M12_list, 'LineWidth',1.5);
title('Moment M_{12} vs \theta_2');
xlabel('\theta_2 [deg]');
ylabel('M_{12} [N-m]');
grid on;

subplot(2,1,2)
plot(theta2_vals, Ms_list, 'LineWidth',1.5);
title('Shaking Moment M_s vs \theta_2');
xlabel('\theta_2 [deg]');
ylabel('M_s [N-m]');
grid on;

