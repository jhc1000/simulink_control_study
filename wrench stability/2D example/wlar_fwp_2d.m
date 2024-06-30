close all;
clear all;
clc;

% 전체 경로 추가
context();

self.robot_name = "wlar_2d";

self.model = wlar_model;
self.kinematics = wlar_kinematics;

self = self.model.init(self);
self = self.kinematics.init(self);

self.q_base = [0.0; 0.0; 0.0];
self.p_base = [0.0; 0.0; 0.0];

self.dot_q_base = [0.0; 0.0; 0.0];
self.dot_p_base = [0.0; 0.0; 0.0];

% self.q.hr = [deg2rad(90); deg2rad(-90); deg2rad(-90); deg2rad(90)];
self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
self.q.hp = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
self.q.k = [deg2rad(90); deg2rad(90); deg2rad(-90); deg2rad(-90)];
self.q.asc = zeros(2,1);

self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);

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
        self.leg_wrench_polytope(j,:,i) = (self.psuedo_inverse_jacobian(:,:,i)*self.leg_torque_space(j,:,i).').';
        self.leg_force_polytope(j,:,i) = [self.leg_wrench_polytope(j,1,i) self.leg_wrench_polytope(j,2,i) self.leg_wrench_polytope(j,3,i)];
    end
end

% for i=1:4
%     if i == 1
%         self.leg_force_polytope_total = [self.leg_wrench_polytope(:,1,i) self.leg_wrench_polytope(:,3,i)];
%     else
%         self.leg_force_polytope_total = minkowskiSum(self.leg_force_polytope_total, [self.leg_wrench_polytope(:,1,i) self.leg_wrench_polytope(:,3,i)]);
%     end
% end
% figure;
% plot(self.leg_force_polytope_total)
% axis equal; grid on;
% xlabel("f_x");ylabel("f_z");
%% Contact Wrench Cone

%% plotting
plotting_tools.plot_robot(self);
plotting_tools.plot_force_polytopes(self);
