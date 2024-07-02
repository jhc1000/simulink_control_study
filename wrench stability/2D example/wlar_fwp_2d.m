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
% self.q.hr = [deg2rad(5); deg2rad(-5); deg2rad(-5); deg2rad(5)];
self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
self.q.hp = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
% self.q.hp = [deg2rad(-90); deg2rad(-90); deg2rad(90); deg2rad(90)];
self.q.k = [deg2rad(90); deg2rad(90); deg2rad(-90); deg2rad(-90)];
self.q.asc = zeros(2,1);

self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);

%% force polytope
self.leg_torque_space(:,:,1) = [self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
    -self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
    self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
    self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
    -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
    self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
    -self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
    -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(3)];

self.leg_torque_space(:,:,2) = [self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
    -self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
    self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
    self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
    -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
    self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
    -self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
    -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(3)];

self.leg_torque_space(:,:,3) = [self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
    -self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
    self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
    self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
    -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
    self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
    -self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
    -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(3)];

self.leg_torque_space(:,:,4) = [self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
    -self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
    self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
    self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
    -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
    self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
    -self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
    -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(3)];

for i=1:4
    self.psuedo_inverse_jacobian(:,:,i) = (self.Jacobian_b(:,:,i)*self.Jacobian_b(:,:,i).')\self.Jacobian_b(:,:,i);
    for j=1:8
        self.leg_force_wrench_polytope(j,:,i) = (self.psuedo_inverse_jacobian(:,:,i)*self.leg_torque_space(j,:,i).').';
        self.leg_force_polytope(j,:,i) = [self.leg_force_wrench_polytope(j,1,i) self.leg_force_wrench_polytope(j,2,i) self.leg_force_wrench_polytope(j,3,i)];
    end
end
%% GRF estimation

%% Contact Wrench Cone
self.leg_friction_space(:,:,1) = 10000.*[self.mu self.mu 1.0;
                                  -self.mu self.mu 1.0;
                                  self.mu -self.mu 1.0;
                                  -self.mu -self.mu 1.0;
                                  0.0 0.0 0.0];

self.leg_friction_space(:,:,2) = 10000.*[self.mu self.mu 1.0;
                                  -self.mu self.mu 1.0;
                                  self.mu -self.mu 1.0;
                                  -self.mu -self.mu 1.0;
                                  0.0 0.0 0.0];

self.leg_friction_space(:,:,3) = 10000.*[self.mu self.mu 1.0;
                                  -self.mu self.mu 1.0;
                                  self.mu -self.mu 1.0;
                                  -self.mu -self.mu 1.0;
                                  0.0 0.0 0.0];

self.leg_friction_space(:,:,4) = 10000.*[self.mu self.mu 1.0;
                                  -self.mu self.mu 1.0;
                                  self.mu -self.mu 1.0;
                                  -self.mu -self.mu 1.0;
                                  0.0 0.0 0.0];

for i=1:4
    for j=1:5
        self.leg_contact_wrench_polytope(j,:,i) = self.leg_friction_space(j,:,i).';
        self.leg_friction_polytope(j,:,i) = [self.leg_contact_wrench_polytope(j,1,i) self.leg_contact_wrench_polytope(j,2,i) self.leg_contact_wrench_polytope(j,3,i)];
    end
end

%% Total Force Polytope
for i=1:4
    if i == 1
        self.leg_force_polytope_total = [self.leg_force_wrench_polytope(:,1,i) self.leg_force_wrench_polytope(:,5,i) self.leg_force_wrench_polytope(:,3,i)];
    else
        self.leg_force_polytope_total = geometry_computation.minkowskiSum(self.leg_force_polytope_total,[self.leg_force_wrench_polytope(:,1,i) self.leg_force_wrench_polytope(:,5,i) self.leg_force_wrench_polytope(:,3,i)]);
    end
end

% Compute the convex hull of the Minkowski sum
K1 = convhull(self.leg_force_polytope_total(:, 1), self.leg_force_polytope_total(:, 2), self.leg_force_polytope_total(:, 3));

% Plot the result
% figure;
% trisurf(K1, self.leg_force_polytope_total(:,1),self.leg_force_polytope_total(:,2),self.leg_force_polytope_total(:,3), 'FaceColor', 'green');
% xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
% title('Convex Hull of Total Force Polytope');
% axis equal;

%% Total Friction Polytope
for i=1:4
    if i == 1
        self.leg_friction_polytope_total = [self.leg_contact_wrench_polytope(:,1,i) self.leg_contact_wrench_polytope(:,2,i) self.leg_contact_wrench_polytope(:,3,i)];
    else
        self.leg_friction_polytope_total = geometry_computation.minkowskiSum(self.leg_friction_polytope_total,[self.leg_contact_wrench_polytope(:,1,i) self.leg_contact_wrench_polytope(:,2,i) self.leg_contact_wrench_polytope(:,3,i)]);
    end
end

% Compute the convex hull of the Minkowski sum
K2 = convhull(self.leg_friction_polytope_total(:, 1), self.leg_friction_polytope_total(:, 2), self.leg_friction_polytope_total(:, 3));

% Plot the result
% figure;
% trisurf(K2, self.leg_friction_polytope_total(:,1),self.leg_friction_polytope_total(:,2),self.leg_friction_polytope_total(:,3), 'FaceColor', 'red');
% xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
% title('Convex Hull of Total Friction Polytope');
% axis equal;

%% Total Feasible force Polytope
% self.leg_feasible_polytope_total = geometry_computation.minkowskiSum(self.leg_force_polytope_total,self.leg_friction_polytope_total);

% Create polyhedra from vertices
P1 = Polyhedron(self.leg_force_polytope_total);
P2 = Polyhedron(self.leg_friction_polytope_total);

% Compute the intersection of the two polyhedra
intersection = intersect(P1, P2);

% Plot the first convex hull
figure
subplot(1,3,1)
P1.plot('color', 'green', 'alpha', 0.5);
title('Force Polytope')
xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
axis equal
view(45,45);

% Plot the second convex hull
subplot(1,3,2)
P2.plot('color', 'red', 'alpha', 0.5);
title('Friction Polytope')
xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
axis equal
view(45,45);

% Plot the intersection
subplot(1,3,3)
intersection.plot('color', 'blue', 'alpha', 0.5);
title('Feasible Polytope')
xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
axis equal
view(45,45);


%% plotting
plotting_tools.plot_robot_space(self);
% plotting_tools.plot_robot_base(self);
% plotting_tools.plot_force_polytopes(self);
% plotting_tools.plot_friction_polytopes(self);
