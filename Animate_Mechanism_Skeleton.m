clear; clc; close all;

%% Known lengths
AB = 36;    % cm
AC = 60;    % cm
BD = 120;   % cm

%% Coordinate of A is (0,0)
A = [0, 0];

%% Set up figure
figure('Name','Slider-Block Mechanism','NumberTitle','off');
axis equal; grid on; hold on;
xlabel('X (cm)'); ylabel('Y (cm)');
title('Mechanism Animation: C on BD, D slides horizontally');
axis([-50 150 -10 50]);  % adjust as needed

%% Animate over a range of input angles theta2
theta2_range = linspace(0, 2*360, 180);  % 0 to 720 deg, 180 frames

% Plot the ground pivot A
plot(A(1), A(2), 'ks','MarkerSize',8,'MarkerFaceColor','k');

for k = 1:length(theta2_range)
    theta2 = theta2_range(k);  % degrees

    % ------------------------------------------------------------
    % 1) Point B from input link AB rotating about A
    %    B = A + [AB*cos(theta2), AB*sin(theta2)]
    % ------------------------------------------------------------
    Bx = AB*cosd(theta2);
    By = AB*sind(theta2);
    B  = [Bx, By];

    % ------------------------------------------------------------
    % 2) Solve for xD, the horizontal position of D, given BD=120
    %    D = (xD, 8.4).
    %    This often requires your loop‐closure:
    %      sqrt( (Bx - xD)^2 + (By - 8.4)^2 ) = 120
    %    => xD can have two possible solutions, pick the one that
    %       makes sense for your mechanism’s motion.
    % ------------------------------------------------------------
    % Example direct solution:
    %   BD^2 = (Bx - xD)^2 + (By - 8.4)^2
    %   => xD = Bx ± sqrt( BD^2 - (By - 8.4)^2 )
    % Adjust sign or logic as needed.

    val = BD^2 - (By - 8.4)^2;    % expression under sqrt
    if val < 0
        % If geometry says no real solution, skip or break
        disp('No real solution for xD at this theta2!');
        continue;
    end
    xD_candidate = Bx + sqrt(val);   % pick plus or minus
    % (You might choose minus for the other branch)
    xD = xD_candidate;
    D  = [xD, 8.4];

    % ------------------------------------------------------------
    % 3) Point C: a slider on BD, and also AC=60
    %    We know line BD is from B to D, so param eqn:
    %        C = B + lambda (D - B),  for 0 <= lambda <= 1
    %    Then enforce AC=60 to solve for lambda.
    % ------------------------------------------------------------
    %   AC^2 = norm( A - C )^2 => 60^2
    %   but C = B + lambda*(D-B).
    %
    %   Let DB = D - B = (Dx - Bx, 8.4 - By).
    %   Then A->C is A->B + lambda*(D-B).
    %   So A->C = (Bx,By) + lambda(DB) - (0,0).
    %   => (Bx + lambda*DBx, By + lambda*DBy).
    %
    %   We want: (Bx + lambda*DBx)^2 + (By + lambda*DBy)^2 = 60^2.
    %   Solve for lambda.  Then C = B + lambda*(D-B).
    %

    DB = D - B;            % Vector from B to D
    DBx = DB(1); DBy = DB(2);

    % Quadratic in lambda
    %   (Bx + lambda*DBx)^2 + (By + lambda*DBy)^2 = 60^2
    A_qu = DBx^2 + DBy^2;
    B_qu = 2*(Bx*DBx + By*DBy);
    C_qu = Bx^2 + By^2 - AC^2;  % AC=60 -> AC^2=3600

    lambda_sol = roots([A_qu, B_qu, C_qu]);
    % Typically two possible solutions; pick the one that matches
    % your “slider on BD” (0 <= lambda <= 1 if C is between B and D).
    lam_candidates = lambda_sol( lambda_sol>=0 & lambda_sol<=1 );
    if isempty(lam_candidates)
        % no solution in [0,1], possibly outside the segment
        disp('No valid lambda in [0,1]!');
        continue;
    end
    lambda = lam_candidates(1);  % pick first if multiple

    C = B + lambda*DB;  % final point C

    % ------------------------------------------------------------
    % Plot links: (A->B), (B->C), (C->A) for the first loop,
    % and B->D for the second loop, or maybe D->C if you prefer
    % to show that slider segment distinctly.
    % ------------------------------------------------------------
    linkPlot1 = plot([A(1), B(1)], [A(2), B(2)], 'ro-','LineWidth',2);
    linkPlot2 = plot([B(1), C(1)], [B(2), C(2)], 'mo-','LineWidth',2);
    linkPlot3 = plot([A(1), C(1)], [A(2), C(2)], 'co-','LineWidth',2);
    linkPlot4 = plot([B(1), D(1)], [B(2), D(2)], 'ko-','LineWidth',2);

    % Show the points B, C, D themselves
    ptB = plot(B(1), B(2), 'bo','MarkerFaceColor','b');
    ptC = plot(C(1), C(2), 'gs','MarkerFaceColor','g');
    ptD = plot(D(1), D(2), 'kd','MarkerFaceColor','k');

    pause(0.05);  % adjust speed

    % Clean up before next frame
    delete(linkPlot1); delete(linkPlot2); delete(linkPlot3); 
    delete(linkPlot4); delete(ptB); delete(ptC); delete(ptD);
end

hold off;
