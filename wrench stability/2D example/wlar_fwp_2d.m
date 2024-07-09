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
% self.p_base = [-2.0; -1.972; 0.4594];
self.p_base = [-2.0; -1.972; 0.4879];
% self.p_base = [-4.0; -2.5; 0.4594];

self.dot_q_base = [0.0; 0.0; 0.0];
self.dot_p_base = [0.0; 0.0; 0.0];

self.q.hr = [deg2rad(90); deg2rad(-90); deg2rad(-90); deg2rad(90)];
% self.q.hr = [deg2rad(60); deg2rad(-60); deg2rad(-60); deg2rad(60)];
% self.q.hr = [deg2rad(30); deg2rad(-30); deg2rad(-30); deg2rad(30)];
% self.q.hr = [deg2rad(45); deg2rad(-45); deg2rad(-45); deg2rad(45)];
% self.q.hr = [deg2rad(1); deg2rad(-1); deg2rad(-1); deg2rad(1)];
% self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
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
m = self.model.totalmass;
g = 9.8.*math_tools.rpyToRot(self.slope(1), self.slope(2), self.slope(3))*[0; 0; 1];
self.resultant_force.b = m.*g;
self.resultant_moment.b = zeros(3,1);
self.resultant_wrench.b = [self.resultant_force.b; self.resultant_moment.b];
self.resultant_wrench_3d = [self.resultant_wrench.b(1), self.resultant_wrench.b(5), self.resultant_wrench.b(3)];

%% Contact Wrench Cone
self = geometry_computation.compute_contact_wrench_polytope(self);

%% Ascender Wrench Polytope
self = geometry_computation.compute_ascender_wrench_polytope(self);

self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_contact_wrench_polytope_total_3d, self.asc_wrench_polytope_3d);
self.force_polytope_total_convhull = Polyhedron(self.force_polytope_total_3d);

%% Total Feasible force Polytope

% Create polyhedra from vertices
P1 = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
P2 = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
P3 = Polyhedron(self.force_polytope_total_3d);

% Compute the intersection of the two polyhedra
self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);
self.feasible_wrench_polytope_total1_convhull = intersect(P1, P3);

% 프로젝션 차원
projection_dims_3d = [1, 2, 3];
projection_dims_2d = [1, 2];  % 예시: xy 평면으로 투영

% 초기 폴리토프 프로젝션
ProjectedPolytope3D = self.feasible_wrench_polytope_total1_convhull.projection(projection_dims_3d);
ProjectedPolytope2D = self.feasible_wrench_polytope_total1_convhull.projection(projection_dims_2d);

fig = figure;
subplot(1,3,1);
self.leg_actuation_wrench_polytope_total_convhull.plot('color', 'green', 'alpha', 0.5);
title('Actuation&Tension Wrench Polytope')
xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% axis equal
% xlim([-10000 10000]); ylim([-10000 10000]); zlim([-10000 10000]);
view(-60,10);

% Plot the second convex hull
subplot(1,3,2);
self.force_polytope_total_convhull.plot('color', 'red', 'alpha', 0.5);
title('Contact Wrench Polytope')
xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% axis equal
% xlim([-3000 3000]); ylim([-3000 3000]); zlim([-0 10000]);
view(-60,10);

% % Plot the second convex hull
% subplot(1,4,3);
% Polyhedron(self.ascender_force_polytope).plot('color', 'cyan', 'alpha', 0.5);
% title('Tenstion Wrench Polytope')
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{F_y} \rm{[Nm]}$', 'Interpreter', 'latex');
% zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % axis equal
% % xlim([-3000 3000]); ylim([-3000 3000]); zlim([-0 10000]);
% % view(-45,30);

% Plot the intersection
subplot(1,3,3);
hold on
self.feasible_wrench_polytope_total1_convhull.plot('color', 'blue', 'alpha', 0.5);
plot3(self.resultant_wrench.b(1), self.resultant_wrench.b(5), self.resultant_wrench.b(3),'o','Color','r','Markersize',10);
title('Stable Wrench Polytope')
xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% axis equal;
% xlim([-3000 3000]); ylim([-3000 3000]); zlim([-0 10000]);
view(-60,10);
hold off

%% 2D Region
% Define the support region of the 4-leg contact area

% % Define parameters for the problem
% cxy = [0; 0];           % Initial guess for cxy
% cz = 0;                 % Placeholder for cz (not used in current functions)
% alpha = 0;              % Angle alpha (adjust as needed)
% R_sb = eye(3);          % Rotation matrix (identity for simplicity)
% p = self.p.s_w;
% 
% n = 3;                  % Placeholder for n (not used in current functions)
% mu = 0.5;               % Friction coefficient
% tau_min = -1;           % Minimum torque constraint
% tau_max = 1;            % Maximum torque constraint
% ej = [1 0;              % Directions ej defining the constraints
%       0 1];
% v = [1 1;               % Velocities v defining the constraints
%      -1 1];
% t_min = -1;             % Minimum force constraint
% t_max = 1;              % Maximum force constraint
% epsilon = 0.01;         % Stopping criterion epsilon
% 
% % Initialize the feasible region
% try
%     Yfa = geometry_computation.feasibleRegionIP(self, cxy, cz, alpha, R_sb, p, n, mu, tau_min, tau_max, ej, v, t_min, t_max, epsilon);
% catch ME
%     disp(ME.message);
%     error('Failed to initialize the feasible region. Check input points.');
% end
% 
% % Plot the initial feasible region
% figure;
% hold on;
% plot3(Yfa(1, :), Yfa(2, :), Yfa(3, :), 'b-', 'LineWidth', 2);
% plot3(Yfa(1, :), Yfa(2, :), Yfa(3, :), 'bo', 'MarkerFaceColor', 'b');
% 
% % Label the vertices
% for i = 1:size(Yfa, 2)
%     text(Yfa(1, i), Yfa(2, i), Yfa(3, i), sprintf('P%d', i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
% end
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Initial Feasible Region');
% axis equal;
% grid on;
% 
% % Iterate to update the feasible region and plot each step
% while true
%     % I) Compute the edges of Yinner
%     edges = geometry_computation.computeEdges(Yfa);
% 
%     % II) Pick ai based on the edge cutting off the largest fraction of Youter
%     ai = geometry_computation.pickEdge(edges, Yfa);
% 
%     % III) Solve the LP
%     try
%         [cxy_opt, f_opt] = geometry_computation.solveLP(ai, cxy, alpha, R_sb, p, n, mu, tau_min, tau_max, ej, v, t_min, t_max);
%     catch ME
%         disp(ME.message);
%         break; % Break if no feasible solution found
%     end
% 
%     % IV) Update the outer approximation Youter
%     Yfa = geometry_computation.updateOuterApproximation(Yfa, cxy_opt);
% 
%     % V) Update the inner approximation Yinner
%     Yfa = geometry_computation.updateInnerApproximation(Yfa, cxy_opt, f_opt);
% 
%     % Plot the updated feasible region
%     figure;
%     hold on;
%     plot3(Yfa(1, :), Yfa(2, :), Yfa(3, :), 'b-', 'LineWidth', 2);
%     plot3(Yfa(1, :), Yfa(2, :), Yfa(3, :), 'bo', 'MarkerFaceColor', 'b');
% 
%     % Label the vertices
%     for i = 1:size(Yfa, 2)
%         text(Yfa(1, i), Yfa(2, i), Yfa(3, i), sprintf('P%d', i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
%     end
% 
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     title('Updated Feasible Region');
%     axis equal;
%     grid on;
% 
%     % Check stopping criterion
%     if area(Yfa) <= epsilon
%         break;
%     end
% end



%% Plotting
% plotting_tools.plot_robot_space(self);
plotting_tools.plot_robot_base(self);
% plotting_tools.plot_force_polytopes(self);
% plotting_tools.plot_ascender_force_polytopes(self);
% plotting_tools.plot_friction_polytopes(self);
% plotting_tools.plot_fesible_polytopes(self);
% plotting_tools.plot_fesible_polytopes1(self);

%% Animation
% plotting_tools.animation_fesible_polytopes(self);
% plotting_tools.animation_fesible_polytopes1(self);
% plotting_tools.animation_fk_sim(self);
% plotting_tools.animation_fk_sim_xyz(self);


