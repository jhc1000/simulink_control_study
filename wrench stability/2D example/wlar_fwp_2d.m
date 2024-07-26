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
% self.p_base = [-1.0; -3.0; 0.4879];
% self.p_base = [-4.0; -2.5; 0.4594];

self.dot_q_base = [0.0; 0.0; 0.0];
self.dot_p_base = [0.0; 0.0; 0.0];

self.ddot_p_base = [0.0; 0.0; 0.0];

% self.q.hr = [deg2rad(90); deg2rad(-90); deg2rad(-90); deg2rad(90)];
% self.q.hr = [deg2rad(60); deg2rad(-60); deg2rad(-60); deg2rad(60)];
% self.q.hr = [deg2rad(30); deg2rad(-30); deg2rad(-30); deg2rad(30)];
% self.q.hr = [deg2rad(45); deg2rad(-45); deg2rad(-45); deg2rad(45)];
% self.q.hr = [deg2rad(1); deg2rad(-1); deg2rad(-1); deg2rad(1)];
self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
self.q.hp = [deg2rad(-50); deg2rad(-50); deg2rad(50); deg2rad(50)];
self.q.k = [deg2rad(80); deg2rad(80); deg2rad(-80); deg2rad(-80)];
% self.q.hp = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
% self.q.hp = [deg2rad(-90); deg2rad(-90); deg2rad(90); deg2rad(90)];
% self.q.k = [deg2rad(90); deg2rad(90); deg2rad(-90); deg2rad(-90)];
self.q.asc = zeros(2,1);

%% Kinematics and Jacobians

self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);
self = self.kinematics.ascender_Jacobians(self, self.dot_q_base, self.dot_p_base);

%% Force Wrench Polytope
self = geometry_computation.compute_force_wrench_polytope(self);

%% GRF Estimation
m = self.model.totalmass;
g = 9.8.*math_tools.rpyToRot(self.slope(1), self.slope(2), self.slope(3))*[0; 0; 1];
self.resultant_force.b = m.*g;
self.resultant_moment.b = zeros(3,1);
self.resultant_wrench.b = [self.resultant_force.b; self.resultant_moment.b];
self.resultant_wrench_3d = [self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3)];

self.com_force.b_norm = -self.resultant_force.b/norm(self.resultant_force.b);

% Define the two points
p1 = self.p_base';
p2 = self.p_base'+self.com_force.b_norm';

% Check if both points are on the same side of the z=0 plane
if (p1(3) * p2(3) > 0)
    error('Both points are on the same side of the z=0 plane. No intersection.');
end

% Calculate the parameter t where the line intersects the z=0 plane
t = p1(3) / (p1(3) - p2(3));

% Calculate the intersection point xy plane
self.com_xy_position = p1 + t * (p2 - p1);

%% Contact Wrench Cone
self = geometry_computation.compute_contact_wrench_polytope(self);

%% Ascender Tension Wrench Polytope
self = geometry_computation.compute_ascender_wrench_polytope(self);

self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_contact_wrench_polytope_total, self.ascender_force_polytope);
self.force_polytope_total_convhull = Polyhedron(self.force_polytope_total_3d);

self.actuation_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total, self.ascender_actuation_force_polytope);
self.actuation_polytope_total_convhull = Polyhedron(self.actuation_polytope_total_3d);

%% Ascender Actuation Wrench Polytope

% fig = figure;
% hold on
% plot3(self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3),'.','Color','magenta','Markersize',30);
% quiver3(0,0,0,self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3),'Color','magenta','LineWidth',3);
% self.ascender_actuation_force_polytope_convhull.plot('color', 'cyan', 'alpha', 0.5);
% title('Ascender Actuation Wrench Polytope')
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{F_y} \rm{[N]}$', 'Interpreter', 'latex');
% zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % axis equal
% % xlim([-10000 10000]); ylim([-10000 10000]); zlim([-10000 10000]);
% view(-60,10);
% hold off


%% Total Feasible force Polytope

% % Create polyhedra from vertices
% P1 = Polyhedron(self.leg_actuation_wrench_polytope_total);
% P2 = Polyhedron(self.leg_contact_wrench_polytope_total);
% P3 = Polyhedron(self.force_polytope_total_3d);
% P4 = Polyhedron(self.actuation_polytope_total_3d);
% 
% % Compute the intersection of the two polyhedra
% self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);
% self.feasible_wrench_polytope_total1_convhull = intersect(P1, P3);
% self.feasible_wrench_polytope_total2_convhull = intersect(P3, P4);
% % 프로젝션 차원
% projection_dims_3d = [1, 2, 3];
% projection_dims_2d = [1, 2];  % 예시: xy 평면으로 투영
% 
% % 초기 폴리토프 프로젝션
% ProjectedPolytope3D = self.feasible_wrench_polytope_total1_convhull.projection(projection_dims_3d);
% ProjectedPolytope2D = self.feasible_wrench_polytope_total1_convhull.projection(projection_dims_2d);
% 
% fig = figure;
% subplot(1,3,1);
% hold on
% plot3(self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3),'.','Color','magenta','Markersize',30);
% quiver3(0,0,0,self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3),'Color','magenta','LineWidth',3);
% P4.plot('color', 'green', 'alpha', 0.5);
% title('Actuation Wrench Polytope')
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{F_y} \rm{[N]}$', 'Interpreter', 'latex');
% zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % axis equal
% % xlim([-10000 10000]); ylim([-10000 10000]); zlim([-10000 10000]);
% view(-60,10);
% hold off
% 
% % Plot the second convex hull
% subplot(1,3,2);
% hold on
% plot3(self.resultant_wrench.b(1), self.resultant_wrench.b(5), self.resultant_wrench.b(3),'.','Color','magenta','Markersize',30);
% quiver3(0,0,0,self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3),'Color','magenta','LineWidth',3);
% P3.plot('color', 'red', 'alpha', 0.5);
% title('Contact&Tension Wrench Polytope')
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{F_y} \rm{[N]}$', 'Interpreter', 'latex');
% zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % axis equal
% xlim([-3000 3000]); ylim([-3000 3000]); zlim([-0 10000]);
% view(-60,10);
% hold off
% 
% % % Plot the second convex hull
% % subplot(1,4,3);
% % Polyhedron(self.ascender_force_polytope).plot('color', 'cyan', 'alpha', 0.5);
% % title('Tenstion Wrench Polytope')
% % xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% % ylabel('$\it{F_y} \rm{[Nm]}$', 'Interpreter', 'latex');
% % zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % % axis equal
% % % xlim([-3000 3000]); ylim([-3000 3000]); zlim([-0 10000]);
% % % view(-45,30);
% 
% % Plot the intersection
% subplot(1,3,3);
% hold on
% plot3(self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3),'.','Color','magenta','Markersize',30);
% quiver3(0,0,0,self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3),'Color','magenta','LineWidth',3);
% self.feasible_wrench_polytope_total2_convhull.plot('color', 'blue', 'alpha', 0.5);
% title('Stable Wrench Polytope')
% xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
% ylabel('$\it{F_y} \rm{[N]}$', 'Interpreter', 'latex');
% zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
% % axis equal;
% % xlim([-3000 3000]); ylim([-3000 3000]); zlim([-0 10000]);
% view(-60,10);
% hold off

%% 2D Region
% Define the support region of the 4-leg contact area

% % Define parameters for the problem
% cxy = [0; 0];           % Initial guess for cxy
% cz = 0;                 % Placeholder for cz (not used in current functions)
% alpha = 0;              % Angle alpha (adjust as needed)
% R_sb = eye(3);          % Rotation matrix (identity for simplicity)
% p = self.p.s_w;
% 
% num_contact = 3;  
% bool_contact = [1, 1, 1, 0];  
% mu = 0.5;               % Friction coefficient
% tau_min = -self.model.LF_tau_lim;           % Minimum torque constraint
% tau_max = self.model.LF_tau_lim;            % Maximum torque constraint
% ej = self.p.s_ej;
% v = [(self.v.b_ej_anc(:,1))/norm(self.v.b_ej_anc(:,1)), (self.v.b_ej_anc(:,2))/norm(self.v.b_ej_anc(:,2))];
% t_min = self.ASC_L_tension_lim(1);             % Minimum force constraint
% t_max =  self.ASC_L_tension_lim(2);            % Maximum force constraint
% epsilon = 0.01;         % Stopping criterion epsilon



%% Moment Calculation

% Define the range of n_leg_friction and n_asc_tension
n_leg_friction_range = 1:5;
n_asc_tension_range = 1:5;

% Initialize the zmp array
self.zmp = zeros(3, 4*4*4*4*4);
self.total_moment = zeros(4*4*4*4*4, 3);
self.total_force = zeros(4*4*4*4*4, 3); % Total force array


% Calculation of ZMP
for n_leg_friction_1 = 1:4
    for n_leg_friction_2 = 1:4
        for n_leg_friction_3 = 1:4
            for n_leg_friction_4 = 1:4
                for n_asc_tension = 1:4
                    num_point = 4*4*4*4*(n_leg_friction_1-1)+4*4*4*(n_leg_friction_2-1)+4*4*(n_leg_friction_3-1)+4*(n_leg_friction_4-1)+(n_asc_tension);

                    points = [self.p.s_w(:,1)';
                              self.p.s_w(:,2)';
                              self.p.s_w(:,3)';
                              self.p.s_w(:,4)';
                              self.p.s_ej(:,1)';
                              self.p.s_ej(:,2)'];
                    forces = [self.leg_contact_force_polytope(n_leg_friction_1,:,1);
                              self.leg_contact_force_polytope(n_leg_friction_2,:,2);
                              self.leg_contact_force_polytope(n_leg_friction_3,:,3);
                              self.leg_contact_force_polytope(n_leg_friction_4,:,4);
                              (self.asc_wrench_matrix(1:3,1)*self.asc_tension_space(n_asc_tension,1))';
                              (self.asc_wrench_matrix(1:3,2)*self.asc_tension_space(n_asc_tension,2))'];

                    % Add inertia forces due to base acceleration
                    inertia_forces = self.model.totalmass * self.ddot_p_base; % F = ma
                    forces = [forces; inertia_forces'; self.com_force.b_norm'];

                    % Calculate moments and forces
                    for i = 1:8 % Updated to 7 to include inertia forces
                        if i <= 6
                            r = points(i, :);   % Position vector of the point
                        else
                            r = [0.0, 0.0, 0.0]; % Base center of mass assumed at origin
                        end
                        F = forces(i, :);   % Force vector at the point
                        M = cross(r, F);    % Moment generated by the force at this point
                        self.total_moment(num_point,:) = self.total_moment(num_point,:) + M; % Sum the moments
                        self.total_force(num_point,:) = self.total_force(num_point,:) + F;   % Sum the forces
                    end

                    % Calculate ZMP (assuming ZMP is on the xy-plane, i.e., z = 0)
                    if self.total_force(num_point, 3) ~= 0
                        self.zmp(1, num_point) = -self.total_moment(num_point, 2) / self.total_force(num_point, 3); % ZMP x-coordinate
                        self.zmp(2, num_point) =  self.total_moment(num_point, 1) / self.total_force(num_point, 3); % ZMP y-coordinate
                        self.zmp(3, num_point) = 0; % ZMP z-coordinate (xy-plane)
                    else
                        % Handle case where the vertical force is zero to avoid division by zero
                        self.zmp(:, num_point) = [NaN; NaN; NaN];
                    end
                end
            end
        end
    end
end

% Extract x and y components for plotting
x_zmp = squeeze(self.zmp(1, :));
y_zmp = squeeze(self.zmp(2, :));

% Remove NaN values if any
valid_indices = ~isnan(x_zmp) & ~isnan(y_zmp);
x_zmp = x_zmp(valid_indices);
y_zmp = y_zmp(valid_indices);

% Plot using scatter
figure;
scatter(x_zmp, y_zmp, 'filled');

xlabel('x (ZMP Position)');
ylabel('y (ZMP Position)');
title('ZMP Positions on the xy-plane');
grid on;
axis equal;

figure;
% self.zmp_polytope = Polyhedron([self.p_base(1), self.p_base(2), 0.0]+self.zmp');
self.zmp_polytope = Polyhedron(self.zmp');
self.zmp_polytope.plot('color', 'gray', 'alpha', 0.5);


%% Plotting
plotting_tools.plot_robot_space(self);
% plotting_tools.plot_robot_base(self);
% plotting_tools.plot_force_polytopes(self);
% plotting_tools.plot_ascender_force_polytopes(self);
% plotting_tools.plot_friction_polytopes(self);
plotting_tools.plot_fesible_polytopes(self);
plotting_tools.plot_fesible_polytopes1(self);

%% Animation
% plotting_tools.animation_fesible_polytopes(self);
% plotting_tools.animation_fesible_polytopes1(self);
% plotting_tools.animation_fk_sim(self);
% plotting_tools.animation_fk_sim_xyz(self);


