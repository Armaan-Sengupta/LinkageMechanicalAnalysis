function ma = get_ma_vector_Skeleton( theta_i, dtheta_i, ddtheta_i, ...
                             r_i, ddr_i)
    % get_ma_vector: Returns the inertia (mass-acceleration) vector
    % for the links in the mechanism.
    
    %% --- LINK 2 PARAMETERS ---
    R2      = r_i(2);        % reference length for link 2
    th2     = theta_i(2);    % angle of link 2
    w2      = dtheta_i(2);   % angular velocity

    m2 = 2700 * pi * R2;

    % Acceleration of link 2's center of mass
    a_G2x = -0.5 * R2 * (w2^2) * cos(th2);
    a_G2y = -0.5 * R2 * (w2^2) * sin(th2);

    % Inertia forces and moment (Link 2)
    Fx_link2 = m2 * a_G2x; 
    Fy_link2 = m2 * a_G2y;
    M_link2  = 0;  % Because alpha2 = 0


    %% --- LINK 3 PARAMETERS ---
    R3      = r_i(3);        % reference length for link 3
    th3     = theta_i(3);    % angle of link 3
    w3      = dtheta_i(3);   % angular velocity
    alpha3  = ddtheta_i(3);  % angular acceleration
    

    % Mass of Link 3
    m3 = 2700 * pi * R3;
    
    I3 = m3 * (25*1e-6 + (1/12) * R3^2);   % moment of inertia of link 3 about its center of mass

    % Acceleration of link 3's center of mass
    a_G3x = - R2 * (w2^2) * cos(th2) + 0.5 * R3 * (-alpha3 * sin(th3) - w3^2 * cos(th3));
    a_G3y = - R2 * (w2^2) * sin(th2) + 0.5 * R3 * (alpha3 * sin(th3) - w3^2 * sin(th3));

    
    %% Inertial “forces” and moment for Link 3
    Fx_link3 = m3 * a_G3x;
    Fy_link3 = m3 * a_G3y;
    M_link3  = I3 * alpha3;

    %% --- LINK 4 PARAMETERS (Point Mass) ---
    ddR4    = ddr_i(4);  % Linear acceleration of Link 4

    % Mass of Link 4
    m4 = 5;

    % Inertia forces for Link 4 (Point mass has no moment equation)
    Fx_link4 = -m4 * ddR4;  % From ∑Fx = m4 * a_G4x
    Fy_link4 = 0;           % No acceleration in y-direction

    %% --- LINK 5 PARAMETERS (Point Mass) ---
    R6      = r_i(6);
    th6     = theta_i(6);    
    w6      = dtheta_i(6);   
    alpha6  = ddtheta_i(6);  

    % Mass of Link 5
    m5 = 5;  

    % Acceleration of Link 5's center of mass
    a_G5x = R6 * (-alpha6 * sin(th6) - w6^2 * cos(th6));
    a_G5y = R6 * (alpha6 * cos(th6) - w6^2 * sin(th6));

    % Inertial forces for Link 5
    Fx_link5 = m5 * a_G5x;
    Fy_link5 = m5 * a_G5y;
    
    %% --- LINK 6 PARAMETERS (Fixed-Axis Rotation) ---
    R6      = r_i(6);        
    th6     = theta_i(6);    
    w6      = dtheta_i(6);   
    alpha6  = ddtheta_i(6);  

    % Mass of Link 6
    m6 = 2700 * pi * R6;
    
    % Moment of inertia of Link 6 about its center of mass
    I6 = m6 * (25*1e-6 + (1/12) * R6^2);

    % Acceleration of link 6's center of mass
    a_G6x = -alpha6 * sin(th6) - 0.5 * R6 * (w6^2) * cos(th6);
    a_G6y =  alpha6 * cos(th6) - 0.5 * R6 * (w6^2) * sin(th6);

    % Inertial forces and moment for Link 6
    Fx_link6 = m6 * a_G6x;
    Fy_link6 = m6 * a_G6y;
    M_link6  = I6 * alpha6;

    %% --- CONSTRUCT THE FULL INERTIA VECTOR ---
    ma = [
        Fx_link2;
        Fy_link2;
        M_link2;
        Fx_link3;
        Fy_link3;
        M_link3;
        Fx_link4;
        Fy_link4;
        Fx_link5;
        Fy_link5;
        Fx_link6;
        Fy_link6;
        M_link6;
    ];
end
