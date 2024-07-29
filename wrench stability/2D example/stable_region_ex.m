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

self.slope = [0.0, deg2rad(10), 0.0];

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
%
% self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_contact_wrench_polytope_total, self.ascender_force_polytope);
% self.force_polytope_total_convhull = Polyhedron(self.force_polytope_total_3d);
%
% self.actuation_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total, self.ascender_actuation_force_polytope);
% self.actuation_polytope_total_convhull = Polyhedron(self.actuation_polytope_total_3d);


%% 2D Region
% Define the support region of the 4-leg contact area

ai = [1.0;-1.0];

% Define parameters for the problem
% cxy = [0.0;0.0];           % Initial guess for cxy
% cz = 0.0;                 % Placeholder for cz (not used in current functions)
cxy = [self.p_base(1);self.p_base(2)];           % Initial guess for cxy
cz = self.p_base(3);                 % Placeholder for cz (not used in current functions)
R_sb = eye(3);          % Rotation matrix (identity for simplicity)
R_gb = math_tools.rpyToRot(self.slope(1), self.slope(2), self.slope(3));
p = self.p.s_w(:,1:3);
% p = self.p.b_w;

num_contact = 4;
bool_contact = [1, 1, 1, 1];

mu = 0.8;               % Friction coefficient

tau_min = -self.model.LF_tau_lim;           % Minimum torque constraint
tau_max = self.model.LF_tau_lim;            % Maximum torque constraint

ej = self.p.s_ej;
% ej = self.p.b_ej;
v = [(self.v.b_ej_anc(:,1))/norm(self.v.b_ej_anc(:,1)), (self.v.b_ej_anc(:,2))/norm(self.v.b_ej_anc(:,2))];

t_min = self.ASC_L_tension_lim(1);             % Minimum force constraint
t_max =  self.ASC_L_tension_lim(2);            % Maximum force constraint

epsilon = 0.01;         % Stopping criterion epsilon


P = [1 0 0; 0 1 0];
A1 = [];
A2 = [];
A3 = [];
B = [];
G = [];
W = [];
u = [];
d = [];
w = [];

% A1
A1 = [zeros(3,2); m.*cross(repmat(g, 1, 2), P.')];

% A2
for i=1:width(p)
    A_upperbar(:,:,i) = [eye(3); math_tools.skew(p(:,i))];
    A2 = horzcat(A2,A_upperbar(:,:,i));
end

% A3
for i=1:width(ej)
    A_underbar(:,:,i) = [eye(3); math_tools.skew(ej(:,i))];
    A3 = horzcat(A3,A_underbar(:,:,i));
end

% u
u = [m.*g; zeros(3,1)];

% B
b = [([1;0;0]-mu*[0;0;1]).';
    ([0;1;0]-mu*[0;0;1]).';
    -([1;0;0]+mu*[0;0;1]).';
    -([0;1;0]+mu*[0;0;1]).'];
for i=1:width(p)
    B = blkdiag(B,b);
end

% G
for i=1:width(p)
    G_i = [self.Jacobian_b(1:3,:,i)'; -self.Jacobian_b(1:3,:,i)'];
    G = blkdiag(G,G_i);
end

% d
for i=1:width(p)
    d_i = [tau_max, tau_min].';
    d = vertcat(d, d_i);
end

% W
for i=1:width(ej)
    W_i = [v(:,i).'; v(:,i).'];
    W = blkdiag(W,W_i);
end

% w
for i=1:width(ej)
    w_i = [t_max, t_min].';
    w = vertcat(w, w_i);
end

num_cxy = size(cxy, 1);
num_f = size(A2, 2);
num_t = size(A3, 2);

% Objective function: -ai.' * cxy (maximize ai.' * cxy -> minimize -ai.' * cxy)
% f_obj = [-ai; zeros(num_f + num_t, 1)];
f_obj = [-ai; zeros(num_f, 1)];

% Constraints: Aeq * x = beq
% Aeq = [A1, A2, A3];
Aeq = [A1, A2];
beq = u;

% Inequality constraints: A * x <= b
% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2)+size(W,2));
%     zeros(size(B,1),size(cxy,1)),B,zeros(size(B,1),size(W,2));
%     zeros(size(G,1),size(cxy,1)),G,zeros(size(G,1),size(W,2));
%     zeros(size(W,1),size(cxy,1)),zeros(size(W,1),size(G,2)),W];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     zeros(size(B, 1), 1);
%     d;
%     w];

A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2));
    zeros(size(B,1),size(cxy,1)),B];

b_ineq = [zeros(size(cxy,1),1);
    zeros(size(B, 1), 1)];

lb = [];
ub = [];

options = optimoptions('linprog','Algorithm','dual-simplex');
[x,fval,exitflag,output] = linprog(f_obj,A_ineq,b_ineq,Aeq,beq,lb,ub,options)

self.com_position_lp = [x(1); x(2); 0.0];

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
                    for i = 1:8 % Updated to 8 to include inertia forces
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
%
% % Plot using scatter
% figure;
% scatter(x_zmp, y_zmp, 'filled');
%
% xlabel('x (ZMP Position)');
% ylabel('y (ZMP Position)');
% title('ZMP Positions on the xy-plane');
% grid on;
% axis equal;

% figure;
% self.zmp_polytope = Polyhedron([self.p_base(1), self.p_base(2), 0.0]+self.zmp');
self.zmp_polytope = Polyhedron(self.zmp');
% self.zmp_polytope.plot('color', 'gray', 'alpha', 0.5);


%% Plotting
plotting_tools.plot_robot_space(self);
% plotting_tools.plot_robot_base(self);
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


