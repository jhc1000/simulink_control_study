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

%% plotting
plotting_tools.plot_robot(self);
plotting_tools.plot_force_polytopes(self);
