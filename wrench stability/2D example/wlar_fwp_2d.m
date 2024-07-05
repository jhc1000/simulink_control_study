close all;
clear all;
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
self.p_base = [-2.0; -1.972; 0.4594];
% self.p_base = [-4.0; -2.5; 0.4594];

self.dot_q_base = [0.0; 0.0; 0.0];
self.dot_p_base = [0.0; 0.0; 0.0];

% self.q.hr = [deg2rad(90); deg2rad(-90); deg2rad(-90); deg2rad(90)];
% self.q.hr = [deg2rad(60); deg2rad(-60); deg2rad(-60); deg2rad(60)];
% self.q.hr = [deg2rad(30); deg2rad(-30); deg2rad(-30); deg2rad(30)];
% self.q.hr = [deg2rad(45); deg2rad(-45); deg2rad(-45); deg2rad(45)];
self.q.hr = [deg2rad(1); deg2rad(-1); deg2rad(-1); deg2rad(1)];
% self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
self.q.hp = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
% self.q.hp = [deg2rad(-90); deg2rad(-90); deg2rad(90); deg2rad(90)];
self.q.k = [deg2rad(90); deg2rad(90); deg2rad(-90); deg2rad(-90)];
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

% 프로젝션 차원
projection_dims_3d = [1, 2, 3];
projection_dims_2d = [1, 2];  % 예시: xy 평면으로 투영

% 초기 폴리토프 프로젝션
ProjectedPolytope3D = self.feasible_wrench_polytope_total1_convhull.projection(projection_dims_3d);
ProjectedPolytope2D = self.feasible_wrench_polytope_total1_convhull.projection(projection_dims_2d);

% % Plot the first convex hull
% fig = figure;
% subplot(1, 4, 1);
% P3.plot('color', 'green', 'alpha', 0.5);
% title('Force Polytope');
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
% zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % axis equal;
% view(-45, 30);
% 
% % Plot the second convex hull
% subplot(1, 4, 2);
% P2.plot('color', 'red', 'alpha', 0.5);
% title('Friction Polytope');
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
% zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % axis equal;
% view(-45, 30);
% 
% % Plot the intersection
% subplot(1, 4, 3);
% self.feasible_wrench_polytope_total1_convhull.plot('color', 'blue', 'alpha', 0.5);
% title('Feasible Polytope');
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
% zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % axis equal;
% view(-45, 30);
% 
% % 2D 프로젝션 서브플롯
% subplot(1, 4, 4);
% ProjectedPolytope2D.plot('color', 'gray', 'alpha', 0.5);
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
% grid on;
% % axis equal;
% view(-90,90);
% title('Projected 2D Wrench Polytope');
% 
% sgtitle('Feasible Wrench Polytope');

%% 2D Region
% Define the support region of the 4-leg contact area
% Assuming support region is defined by vertices (x1, y1), (x2, y2), (x3, y3), (x4, y4)
support_region_vertices = [self.p.b_w(1,1), self.p.b_w(2,1);
    self.p.b_w(1,2), self.p.b_w(2,2);
    self.p.b_w(1,3), self.p.b_w(2,3);
    self.p.b_w(1,4), self.p.b_w(2,4)];

% Project the 3D feasible polytope to the 2D support region plane
ProjectedPolytope2D = self.feasible_wrench_polytope_total1_convhull.projection([1, 2]);

% Plot the initial projections
figure;
subplot(1, 2, 1);
ProjectedPolytope2D.plot('color', 'gray', 'alpha', 0.5);
hold on;
fill(support_region_vertices(:,1), support_region_vertices(:,2), 'r', 'FaceAlpha', 0.1);
xlabel('\it{x [m]}', 'Interpreter', 'latex');
ylabel('\it{y [m]}', 'Interpreter', 'latex');
title('Initial Projected 2D Wrench Polytope and Support Region');
axis equal;
hold off;

% Iteratively refine the feasible region using linear programming
num_vertices = size(ProjectedPolytope2D.V, 1);
feasible_points = [];
for i = 1:num_vertices
    point = ProjectedPolytope2D.V(i, :);
    
    % Linear programming to check if the point lies within the support region
    A = -support_region_vertices;
    b = -ones(size(support_region_vertices, 1), 1);
    f = zeros(size(point));
    options = optimoptions('linprog', 'Display', 'none');
    
    [~, fval, exitflag] = linprog(f, A, b, [], [], point, point, options);
    
    if exitflag == 1  % The point is feasible
        feasible_points = [feasible_points; point];
    end
end

% Plot the refined feasible region
subplot(1, 2, 2);
if ~isempty(feasible_points)
    K = convhull(feasible_points);
    plot(feasible_points(K,1), feasible_points(K,2), 'b-', 'LineWidth', 2);
    hold on;
    fill(support_region_vertices(:,1), support_region_vertices(:,2), 'r', 'FaceAlpha', 0.1);
    xlabel('\it{x [m]}', 'Interpreter', 'latex');
    ylabel('\it{y [m]}', 'Interpreter', 'latex');
    title('Refined Feasible Region in Support Region');
    axis equal;
    hold off;
else
    text(0.5, 0.5, 'No feasible region found within support region', 'HorizontalAlignment', 'center');
end

%% Plotting
plotting_tools.plot_robot_space(self);
% plotting_tools.plot_robot_base(self);
% plotting_tools.plot_force_polytopes(self);
% plotting_tools.plot_ascender_force_polytopes(self);
% plotting_tools.plot_friction_polytopes(self);
% plotting_tools.plot_fesible_polytopes(self);
plotting_tools.plot_fesible_polytopes1(self);

%% Animation
% plotting_tools.animation_fesible_polytopes(self);
% plotting_tools.animation_fesible_polytopes1(self);
% plotting_tools.animation_fk_sim(self);
% plotting_tools.animation_fk_sim_xyz(self);


