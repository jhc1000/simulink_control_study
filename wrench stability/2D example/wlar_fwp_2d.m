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
self.p_base = [-2.0; -1.972; 0.48];

self.dot_q_base = [0.0; 0.0; 0.0];
self.dot_p_base = [0.0; 0.0; 0.0];

% self.q.hr = [deg2rad(90); deg2rad(-90); deg2rad(-90); deg2rad(90)];
% self.q.hr = [deg2rad(60); deg2rad(-60); deg2rad(-60); deg2rad(60)];
% self.q.hr = [deg2rad(30); deg2rad(-30); deg2rad(-30); deg2rad(30)];
% self.q.hr = [deg2rad(45); deg2rad(-45); deg2rad(-45); deg2rad(45)];
% self.q.hr = [deg2rad(1); deg2rad(-1); deg2rad(-1); deg2rad(1)];
self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
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

self.force_polytope_total = geometry_computation.minkowskiSum(self.leg_force_polytope_total, self.ascender_force_polytope);

%% Total Feasible force Polytope
self.leg_feasible_polytope_total = geometry_computation.minkowskiSum(self.leg_force_polytope_total,self.leg_friction_polytope_total);

% Create polyhedra from vertices
P1 = Polyhedron(self.force_polytope_total);
P2 = Polyhedron(self.leg_friction_polytope_total);

% Compute the intersection of the two polyhedra
P3 = intersect(P1, P2);

% Plot the first convex hull
fig = figure;
tiledlayout(1,3);
ax(1) = nexttile;
P1.plot('color', 'green', 'alpha', 0.5);
title(ax(1),'Force Polytope')
xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
axis equal
view(45,30);

% Plot the second convex hull
ax(2) = nexttile;
P2.plot('color', 'red', 'alpha', 0.5);
title(ax(2),'Friction Polytope')
xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
axis equal
view(45,30);

% Plot the intersection
ax(3) = nexttile;
P3.plot('color', 'blue', 'alpha', 0.5);
title(ax(3),'Feasible Polytope')
xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
axis equal
view(45,30);

sgtitle('Feasible Wrench Polytope');


%% Plotting
plotting_tools.plot_robot_space(self);
% plotting_tools.plot_robot_base(self);
% plotting_tools.plot_force_polytopes(self);
plotting_tools.plot_ascender_force_polytopes(self);
% plotting_tools.plot_friction_polytopes(self);
plotting_tools.plot_fesible_polytopes(self);

%% Animation
% plotting_tools.animation_fesible_polytopes(self);




