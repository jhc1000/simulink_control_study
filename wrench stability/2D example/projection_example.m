% Clear workspace and close figures
close all;
clear;
clc;

% 전체 경로 추가
context();

self.robot_name = "wlar_2d";

self.model = wlar_model;
self.kinematics = wlar_kinematics;
self.dynamics = dynamics;

self = self.model.init(self);
self = self.kinematics.init(self);
self = self.dynamics.init(self);

self.slope = [0.0, deg2rad(45), 0.0];

self.anchor.position(:,:,1) = [0.0; 0.0; 0.250];
self.anchor.position(:,:,2) = [0.0; -3.944; 0.250];

self.q_base = [0.0; deg2rad(0); 0.0];
% self.p_base = [-2.0; -1.972; 0.4594];
self.p_base = [-2.0; -1.972; 0.4879];
% self.p_base = [-4.0; -2.5; 0.4594];

self.dot_q_base = [0.0; 0.0; 0.0];
self.dot_p_base = [0.0; 0.0; 0.0];

% self.q.hr = [deg2rad(90); deg2rad(-90); deg2rad(-90); deg2rad(90)];
% self.q.hr = [deg2rad(60); deg2rad(-60); deg2rad(-60); deg2rad(60)];
% self.q.hr = [deg2rad(30); deg2rad(-30); deg2rad(-30); deg2rad(30)];
% self.q.hr = [deg2rad(45); deg2rad(-45); deg2rad(-45); deg2rad(45)];
% self.q.hr = [deg2rad(1); deg2rad(-1); deg2rad(-1); deg2rad(1)];
self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
% self.q.hp = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
self.q.hp = [deg2rad(-50); deg2rad(-50); deg2rad(50); deg2rad(50)];
% self.q.hp = [deg2rad(-90); deg2rad(-90); deg2rad(90); deg2rad(90)];
% self.q.k = [deg2rad(90); deg2rad(90); deg2rad(-90); deg2rad(-90)];
self.q.k = [deg2rad(80); deg2rad(80); deg2rad(-80); deg2rad(-80)];
self.q.asc = zeros(2,1);

%% Kinematics and Jacobians

self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);

%% Force Wrench Polytope
self = geometry_computation.compute_force_wrench_polytope(self);

%% GRF Estimation

%% Contact Wrench Cone
self = geometry_computation.compute_contact_wrench_polytope(self);

%% Ascender Wrench Polytope
self = geometry_computation.compute_ascender_wrench_polytope(self);

self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d, self.asc_wrench_polytope_3d);
self.force_polytope_total_convhull = Polyhedron(self.force_polytope_total_3d);

%% Total Feasible force Polytope

% Create polyhedra from vertices
P1 = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
P2 = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
P3 = Polyhedron(self.force_polytope_total_3d);

% Compute the intersection of the two polyhedra
self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);
self.feasible_wrench_polytope_total1_convhull = intersect(P3, P2);

% Define the points
points = [self.p.b_w(1,:).' self.p.b_w(2,:).'];  % Example points, adjust as needed

% Plot the points
figure;
hold on;
plot(points(:,1), points(:,2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
text(points(:,1), points(:,2), {' P1',' P2',' P3',' P4'},'VerticalAlignment','bottom','HorizontalAlignment','right')
xlabel('X');
ylabel('Y');
title('Points to be Constrained');

% Compute the convex hull of the points
K = convhull(points(:,1), points(:,2));
plot(points(K,1), points(K,2), 'r-', 'LineWidth', 2);
legend('Points', 'Convex Hull', 'Location', 'Best');
axis equal;

% Define constraints and solve using iterative projection and LP
% Example of iterative projection (replace with actual StableRegionComputation class methods)
epsilon = 1e-3;  % Stopping criterion for iterative projection

% Example LP constraints (replace with actual constraints)
ai = [0.0001; 0.0; 0.0];                % Example vector ai
cxy = [0; 0; 0];             % Example initial guess for cxy
alpha = self.slope(2);                 % Example angle alpha
R_sb = eye(3);              % Example rotation matrix R_sb
p = self.p.b_w;             % Example matrix p (3x4 matrix of random numbers)
n = 4;                      % Example number n
mu = 0.3;                   % Example value for mu
tau_min = -100;             % Example minimum value for tau
tau_max = 100;              % Example maximum value for tau
ej = self.p.b_ej;            % Example matrix ej (3x2 matrix of random numbers)
v = self.v.b_ej_anc;             % Example matrix v (3x2 matrix of random numbers)
t_min = 50;                % Example minimum value for t
t_max = 10000;              % Example maximum value for t

% Initialize StableRegionComputation instance
computation = StableRegionComputation(epsilon);

% try
    % Example of iterative projection to find feasible region
    projected_points = iterativeProjection(points, computation);

    % Example of LP to further refine the feasible region
    [cxy_opt, f_opt] = computation.solveLP(self, ai, cxy, alpha, R_sb, p, n, mu, tau_min, tau_max, ej, v, t_min, t_max);
    
    % Plot the results
    plot(projected_points(:,1), projected_points(:,2), 'b.', 'MarkerSize', 15);
    text(projected_points(:,1), projected_points(:,2), {' P1',' P2',' P3',' P4'},'VerticalAlignment','bottom','HorizontalAlignment','right')
    legend('Points', 'Convex Hull', 'Projected Points', 'Location', 'Best');
    
    % Example plot of optimal cxy and f (replace with actual plot based on your data)
    figure;
    % Plotting the optimal cxy
    subplot(1, 2, 1);
    plot(cxy_opt(1), cxy_opt(2), 'ro', 'MarkerSize', 10);
    xlabel('cxy(1)');
    ylabel('cxy(2)');
    title('Optimal cxy');
    axis equal;

    % Plotting the optimal f
    subplot(1, 2, 2);
    bar(f_opt);
    xlabel('Index');
    ylabel('Value');
    title('Optimal f');
    
% catch ME
%     disp(ME.message);
% end

function projected_points = iterativeProjection(points, computation)
    % Example iterative projection method (replace with actual implementation)
    max_iter = 100;
    tol = 1e-6;
    current_points = points;
    for iter = 1:max_iter
        new_points = [];  % Store projected points
        for i = 1:size(current_points, 1)
            % Example projection (replace with actual projection)
            projected_point = current_points(i, :);  % Example: no change
            new_points = [new_points; projected_point];
        end
        current_points = new_points;
        % Example stopping criterion (replace with actual criterion)
        if max(abs(current_points - points)) < tol
            break;
        end
    end
    projected_points = current_points;
end
