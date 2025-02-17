function A = get_A_matrix_Skeleton(theta_i, R2, R3, R3A, R6) 
    A = zeros(13, 13); % Initialize a 13x13 matrix of zeros
    
    % --- LINK 2: Force Equilibrium Equations ---
    % Row 1: ∑F_x for Link 2  =>  F_2x + F_3x = RHS
    A(1,3) =  1;  % F_2x coefficient
    A(1,5) =  1;  % F_3x coefficient
    
    % Row 2: ∑F_y for Link 2  =>  F_2y + F_3y = RHS
    A(2,4) =  1;  % F_2y coefficient
    A(2,6) =  1;  % F_3y coefficient
    
    % Row 3: ∑M_A for Link 2  =>  M_12 - F_3x R2 sin(theta2) + F_3y R2 cos(theta2) = 0
    A(3,13) =  1;  % M_12 coefficient
    A(3,5) = -R2 * sin(theta_i(2));  % -F_3x * R2 * sin(theta2)
    A(3,6) =  R2 * cos(theta_i(2));  % F_3y * R2 * cos(theta2)
    
    % --- LINK 3: Force & Moment Equilibrium Equations ---
    
    % Row 4: ∑F_x for Link 3  =>  -F_3x - F_4x + F5*cos(theta3 - 3pi/2) = 0
    A(4,5) = -1;  % -F_3x coefficient
    A(4,7) = -1;  % -F_4x coefficient
    A(4,9) = cos(theta_i(3) - (3*pi/2));  % F_5 coefficient

    % Row 5: ∑F_y for Link 3  =>  -F_3y + F_4y + F5*sin(theta3 - 3pi/2) = 0
    A(5,6) = -1;  % -F_3y coefficient
    A(5,8) =  1;  % F_4y coefficient
    A(5,9) = sin(theta_i(3) - (3*pi/2));  % F_5 coefficient
    
    % Row 6: ∑M_G3 for Link 3  =>  Moment equation
    A(6,5)  = -0.5 * R3 * sin(theta_i(3));  % -1/2 * F_3x * R3 * sin(theta3)
    A(6,6)  =  0.5 * R3 * cos(theta_i(3));  % 1/2 * F_3y * R3 * cos(theta3)
    A(6,9)  = -(R3 / 2 - R3A);  % -F_5 * (R3/2 - R3A)
    A(6,7)  = -0.5 * R3 * sin(theta_i(3));  % -1/2 * F_4x * R3 * sin(theta3)
    A(6,8)  =  0.5 * R3 * cos(theta_i(3));  % 1/2 * F_4y * R3 * cos(theta3)
    
    % --- LINK 4: Force Equilibrium Equations (Point Mass) ---
    
    % Row 7: ∑F_x for Link 4  =>  -F_4x = RHS (m_4 * ddR4)
    A(7,7) = -1;  % -F_4x coefficient

    % Row 8: ∑F_y for Link 4  =>  -F_4y  + F_14y = 0
    A(8,8) = -1;  % -F_4y coefficient
    A(8,12) = 1; %F_14y coefficient
    
    % --- LINK 5: Force Equilibrium Equations (Point Mass) ---
    
    % Row 9: ∑F_x for Link 5  =>  -F_5 cos(theta3 - 3pi/2) + F_6x = RHS
    A(9,9)  = -cos(theta_i(3) - (3*pi/2));  % -F_5 cos(theta3 - 3pi/2)
    A(9,11) =  1;  % F_6x coefficient

    % Row 10: ∑F_y for Link 5  =>  -F_5 sin(theta3 - 3pi/2) + F_6y = RHS
    A(10,9)  = -sin(theta_i(3) - (3*pi/2));  % -F_5 sin(theta3 - 3pi/2)
    A(10,11) =  1;  % F_6y coefficient
    
    % --- LINK 6: Force & Moment Equilibrium Equations ---
    
    % Row 11: ∑F_x for Link 6  =>  F_6x - F_1x = RHS
    A(11,10) = -1;  % -F_6x coefficient
    A(11,1)  = 1;  % F_1x coefficient

    % Row 12: ∑F_y for Link 6  =>  F_6y - F_1y = RHS
    A(12,11) = -1;  % -F_6y coefficient
    A(12,2)  = 1;  % F_1y coefficient

    % Row 13: ∑M_A for Link 6  =>  F_6x * R6 * sin(theta2) - F_6y * R6 * cos(theta2) = M_6
    A(13,10) =  R6 * sin(theta_i(2));  % F_6x * R6 * sin(theta2)
    A(13,11) = -R6 * cos(theta_i(2));  % -F_6y * R6 * cos(theta2)
    
end
