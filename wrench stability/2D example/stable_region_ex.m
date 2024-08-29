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

self.slope = [0.0, deg2rad(20), 0.0];

self.anchor.position(:,:,1) = [0.0; 0.0; 0.250];
self.anchor.position(:,:,2) = [0.0; -3.944; 0.250];

self.q_base = [0.0; deg2rad(-10); 0.0];
% self.p_base = [-2.0; -1.972; 0.50322];
self.p_base = [-2.0; -1.972; 0.50322];
% self.p_base = [-2.0; -1.972; 0.38545];
% self.p_base = [-1.0; -3.0; 0.50322];
% self.p_base = [-4.0; -2.5; 0.50322];
% self.p_base = [-2.0; -1.0; 0.50322];

self.base_movement = [-0.0; 0.0; 0.0];
% % self.base_movement = [-0.25703; -0.11508; 0.0]; % lf
% self.base_movement = [-0.27; -0.15; 0.0]; % lf2
% self.base_movement = [-0.17997; 0.03313; 0.0];  % rf
% self.base_movement = [0.08638; -0.10347; 0.0];  % lr
% self.base_movement = [0.07631; 0.08888; 0.0];  % rr
% self.base_movement = [0.09299; 0.15955; 0.0];  % rr

self.bool_contact = [0,1,1,1];

self.dot_q_base = [0.0; 0.0; 0.0];
self.dot_p_base = [0.0; 0.0; 0.0];

self.ddot_p_base = [0.0; 0.0; 0.0];

% self.q.hr = [deg2rad(90); deg2rad(-90); deg2rad(-90); deg2rad(90)];
% self.q.hr = [deg2rad(60); deg2rad(-60); deg2rad(-60); deg2rad(60)];
% self.q.hr = [deg2rad(30); deg2rad(-30); deg2rad(-30); deg2rad(30)];
% self.q.hr = [deg2rad(45); deg2rad(-45); deg2rad(-45); deg2rad(45)];
% self.q.hr = [deg2rad(1); deg2rad(-1); deg2rad(-1); deg2rad(1)];
self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
% self.q.hp = [deg2rad(-54.877566198798185); deg2rad(-54.877566198798185); deg2rad(54.877566198798185); deg2rad(54.877566198798185)];
% self.q.k = [deg2rad(81.82706571003209); deg2rad(81.82706571003209); deg2rad(-81.82706571003209); deg2rad(-81.82706571003209)];
self.q.hp = [deg2rad(-50); deg2rad(-50); deg2rad(50); deg2rad(50)];
self.q.k = [deg2rad(80); deg2rad(80); deg2rad(-80); deg2rad(-80)];
% self.q.hp = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
% self.q.k = [deg2rad(75); deg2rad(75); deg2rad(-75); deg2rad(-75)];
% self.q.hr = [deg2rad(45); deg2rad(-45); deg2rad(-45); deg2rad(45)];
% self.q.hp = [deg2rad(-80); deg2rad(-80); deg2rad(80); deg2rad(80)];
% self.q.k = [deg2rad(100); deg2rad(100); deg2rad(-100); deg2rad(-100)];
self.q.asc = zeros(2,1);


%% Kinematics and Jacobians

self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);
self = self.kinematics.ascender_Jacobians(self, self.dot_q_base, self.dot_p_base);

self = self.kinematics.wheelleg_ik(self,self.p.b_w,self.base_movement(1),self.base_movement(2),self.base_movement(3),self.q_base(1),self.q_base(2),self.q_base(3));
self.p_base = self.p_base + self.base_movement;

disp("ik_angle : "+string(self.ik_angle_degree))
self.q.hr = self.ik_angle(:,1);
self.q.hp = self.ik_angle(:,2);
self.q.k = self.ik_angle(:,3);

self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);
self = self.kinematics.ascender_Jacobians(self, self.dot_q_base, self.dot_p_base);

% self = self.kinematics.wheelleg_ik_numeric(self,self.p.b_w,self.base_movement(1),self.base_movement(2),self.base_movement(3),self.q_base(1),self.q_base(2),self.q_base(3));
% disp([self.q.hr, self.q.hp, self.q.k]');

%% Force Wrench Polytope
self = geometry_computation.compute_force_wrench_polytope(self);

%% GRF Estimation
m = self.totalmass;
g = 9.8.*math_tools.rpyToRot(self.slope(1), self.slope(2), self.slope(3))*[0; 0; 1];
self.resultant_force.b = m.*g;
self.resultant_moment.b = zeros(3,1);
self.resultant_wrench.b = [self.resultant_force.b; self.resultant_moment.b];
self.resultant_wrench_3d = [self.resultant_wrench.b(1), self.resultant_wrench.b(2), self.resultant_wrench.b(3)];

self.com_force.b_norm = -self.resultant_force.b/norm(self.resultant_force.b);

% Define the two points
p1 = self.p_base'+[0.115, 0.004, 0.0];
p2 = self.p_base'+[0.115, 0.004, 0.0]+self.com_force.b_norm';

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

ai = [0.0;0.0];

% Define parameters for the problem
cxy = [self.p_base(1);self.p_base(2)];           % Initial guess for cxy
cz = self.p_base(3);                 % Placeholder for cz (not used in current functions)
R_sb = eye(3);          % Rotation matrix (identity for simplicity)
R_gb = math_tools.rpyToRot(self.slope(1), self.slope(2), self.slope(3));

self.num_contact = sum(self.bool_contact);
self.c_bool = logical(self.bool_contact);
p = self.p.s_w(:,self.c_bool);
% disp(p)
% self.mu = 0.6;               % Friction coefficient

tau_min = -self.model.LF_tau_lim;           % Minimum torque constraint
tau_max = self.model.LF_tau_lim;            % Maximum torque constraint

asc_tau_min = -self.ASC_L_tau_lim;
asc_tau_max = self.ASC_L_tau_lim;

ej = self.p.s_ej;
self.v_norm = [(self.v.b_ej_anc(:,1))/norm(self.v.b_ej_anc(:,1)), (self.v.b_ej_anc(:,2))/norm(self.v.b_ej_anc(:,2))];

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
H = [];
u = [];
d = [];
w = [];
h = [];

% A1
A1 = [zeros(3,2); m.*cross(repmat(g, 1, 2), P')];

% A2
for i=1:width(p)
    A_upperbar(:,:,i) = [eye(3); math_tools.skew(p(:,i))];
    A2 = horzcat(A2,A_upperbar(:,:,i));
end

% A3
for i=1:width(ej)
    A_underbar(:,:,i) = [(self.v_norm(:,i)); math_tools.skew(ej(:,i))*self.v_norm(:,i)];
    A3 = horzcat(A3,A_underbar(:,:,i));
end

% u
u = [m.*g; zeros(3,1)];

% B
b = [([1;0;0]-self.mu*[0;0;1])';
    ([0;1;0]-self.mu*[0;0;1])';
    -([1;0;0]+self.mu*[0;0;1])';
    -([0;1;0]+self.mu*[0;0;1])'];
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
    d_i = [tau_max, -tau_min]';
    d = vertcat(d, d_i);
end

% W
% for i=1:width(ej)
%     W_i = [v(:,i)'; -v(:,i)'];
%     W = blkdiag(W,W_i);
% end

for i=1:width(ej)
    W_i = [1; -1];
    W = blkdiag(W,W_i);
end

% w
for i=1:width(ej)
    w_i = [t_max, -t_min]';
    w = vertcat(w, w_i);
end

% H
% for i=1:width(ej)
%     H_i = [v(:,i)'/self.r_sheeve; -v(:,i)'/self.r_sheeve];
%     H = blkdiag(H,H_i);
% end

for i=1:width(ej)
    H_i = [1*self.r_sheeve; -1*self.r_sheeve];
    H = blkdiag(H,H_i);
end

% h
for i=1:width(ej)
    h_i = [asc_tau_max, -asc_tau_min]';
    h = vertcat(h, h_i);
end

num_cxy = size(cxy, 1);
num_f = size(A2, 2);
num_t = size(A3, 2);

% Objective function: -ai.' * cxy (maximize ai.' * cxy -> minimize -ai.' * cxy)
% f_obj = [-ai; -ones(num_f, 1); ones(num_t,1)];
f_obj = [-ai; -ones(num_f, 1)];

% Constraints: Aeq * x = beq
% Aeq = [A1, A2, A3];
Aeq = [A1, A2];
beq = u;

% Inequality constraints: A * x <= b
% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2)+size(W,2));
%     zeros(size(B,1),size(cxy,1)),B,zeros(size(B,1),size(W,2));
%     zeros(size(G,1),size(cxy,1)),G,zeros(size(G,1),size(W,2));
%     zeros(size(W,1),size(cxy,1)),zeros(size(W,1),size(G,2)),W;
%     zeros(size(H,1),size(cxy,1)),zeros(size(H,1),size(G,2)),H];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     zeros(size(B, 1), 1);
%     d;
%     w;
%     h];


% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2)+size(W,2));
%     zeros(size(G,1),size(cxy,1)),G,zeros(size(G,1),size(W,2));
%     zeros(size(W,1),size(cxy,1)),zeros(size(W,1),size(G,2)),W;
%     zeros(size(H,1),size(cxy,1)),zeros(size(H,1),size(G,2)),H];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     d;
%     w;
%     h];


% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2)+size(W,2));
%     zeros(size(B,1),size(cxy,1)),B,zeros(size(B,1),size(W,2));
%     zeros(size(G,1),size(cxy,1)),G,zeros(size(G,1),size(W,2));
%     zeros(size(W,1),size(cxy,1)),zeros(size(W,1),size(G,2)),W];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     zeros(size(B, 1), 1);
%     d;
%     w];

% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2)+size(W,2));
%     zeros(size(B,1),size(cxy,1)),B,zeros(size(B,1),size(W,2));
%     zeros(size(W,1),size(cxy,1)),zeros(size(W,1),size(G,2)),W];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     zeros(size(B, 1), 1);
%     w];

% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2));
%     zeros(size(B,1),size(cxy,1)),B;
%     zeros(size(G,1),size(cxy,1)),G];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     zeros(size(B, 1), 1);
%     d];


% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(G,2));
%     zeros(size(G,1),size(cxy,1)),G];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     d];

A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2));
    zeros(size(B,1),size(cxy,1)),B];

b_ineq = [zeros(size(cxy,1),1);
    zeros(size(B, 1), 1)];

lb = [];
ub = [];


% Define different values for ai
ai_values = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];

% Initialize arrays to store the results
com_position_lp_results = zeros(3, length(ai_values));
grf_results = cell(1, length(ai_values));

% Loop over each ai value
for k = 1:size(ai_values, 1)
    ai = ai_values(k, :)';
    
    % Objective function: -ai.' * cxy (maximize ai.' * cxy -> minimize -ai.' * cxy)
    f_obj = [-ai; -ones(num_f, 1)];
    % f_obj = [-ai; -ones(num_f, 1); ones(num_t,1)];
    
    % Solve the linear program
    [x, fval, exitflag, output] = linprog(f_obj, A_ineq, b_ineq, Aeq, beq, lb, ub);
    
    % Store the resulting com_position_lp and grf
    self.com_position_lp = [x(1); x(2); 0.0];
    % self.grf = x(3:end);
    self.grf = x(3:end-2);
    self.tension_lp = x(end-1:end);
    
    % Save results to arrays
    self.com_position_lp_results(:, k) = self.com_position_lp;
    self.grf_results{k} = self.grf;
    
    % Display results for each iteration
    % disp(['Iteration ', num2str(k), ' for ai = [', num2str(ai(1)), ',', num2str(ai(2)), ']']);
    % disp('com_position_lp:');
    % disp(self.com_position_lp);
    % disp('grf:');
    % disp(self.grf);
    % disp('G * grf:');
    % disp(G * self.grf);
    % disp(self.tension_lp);
    % disp(W * self.tension_lp);
end
%% Moment Calculation
% Calculation of ZMP

% Compute the number of possible configurations
num_configurations = 4.^(self.num_contact+1);
self.zmp = zeros(3, num_configurations);
self.total_moment = zeros(num_configurations, 3);
self.total_force = zeros(num_configurations, 3); % Total force array

% Determine which legs are in contact
contact_indices = find(self.bool_contact);

% Calculation of ZMP
for n_leg_friction_1 = 1:4
    for n_leg_friction_2 = 1:4
        for n_leg_friction_3 = 1:4
            for n_leg_friction_4 = 1:4
                for n_asc_tension = 1:4
                    % Determine the active friction indices
                    active_friction_indices = [n_leg_friction_1, n_leg_friction_2, n_leg_friction_3, n_leg_friction_4];
                    
                    % Compute num_point carefully
                    num_point = 4^(self.num_contact-0) * (n_leg_friction_1 - 1) ...
                              + 4^(self.num_contact-1) * (n_leg_friction_2 - 1) ...
                              + 4^(self.num_contact-2) * (n_leg_friction_3 - 1) ...
                              + 4^(self.num_contact-3) * (n_leg_friction_4 - 1) ...
                              + n_asc_tension;
                    % disp(num_point)

                    % Ensure num_point is within bounds
                    if num_point > num_configurations || num_point < 1 || floor(num_point) ~= num_point
                        % warning('num_point %d is out of bounds or not an integer', num_point);
                        continue; % Skip this iteration if out of bounds or invalid
                    end

                    points = [self.p.s_w(:,1)';
                              self.p.s_w(:,2)';
                              self.p.s_w(:,3)';
                              self.p.s_w(:,4)';
                              self.p.s_ej(:,1)';
                              self.p.s_ej(:,2)'];
                    forces = zeros(8, 3);
                    
                    % Assign forces based on contact
                    for leg_index = 1:4
                        forces(leg_index, :) = self.leg_contact_force_polytope(active_friction_indices(leg_index), :, leg_index) .* self.bool_contact(leg_index);
                    end
                    forces(5, :) = (self.asc_wrench_matrix(1:3,1) * self.asc_tension_space(n_asc_tension,1))';
                    forces(6, :) = (self.asc_wrench_matrix(1:3,2) * self.asc_tension_space(n_asc_tension,2))';
                    
                    % Add inertia forces due to base acceleration
                    inertia_forces = self.model.totalmass * self.ddot_p_base; % F = ma
                    forces = [forces; inertia_forces'; self.com_force.b_norm'];

                    % Calculate moments and forces
                    for i = 1:8 % Updated to 8 to include inertia forces
                        if i <= 6
                            r = points(i, :);   % Position vector of the point
                        else
                            r = self.p_base' + [0.115, 0.004, 0.0]; % Base center of mass assumed at origin
                        end
                        F = forces(i, :);   % Force vector at the point
                        M = cross(r, F);    % Moment generated by the force at this point
                        self.total_moment(num_point, :) = self.total_moment(num_point, :) + M; % Sum the moments
                        self.total_force(num_point, :) = self.total_force(num_point, :) + F;   % Sum the forces
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

%% function etc
% Helper function to recursively generate nested for loops
function recursive_for(self, depth, index, num_point)
    disp("depth : " + string(depth) + ", index : " + mat2str(index) + ", num_point : " + string(num_point))
    
    if depth > length(self.bool_contact)
        % Base case: This will only be reached when all the necessary loops have been generated.
        
        for n_asc_tension = 1:4
            % Calculate the forces based on the current index and n_asc_tension
            points = [self.p.s_w(:,1)';
                        self.p.s_w(:,2)';
                        self.p.s_w(:,3)';
                        self.p.s_w(:,4)';
                        self.p.s_ej(:,1)';
                        self.p.s_ej(:,2)'];

            forces = [
                self.leg_contact_force_polytope(index(1),:,1) .* self.bool_contact(1);
                self.leg_contact_force_polytope(index(2),:,2) .* self.bool_contact(2);
                self.leg_contact_force_polytope(index(3),:,3) .* self.bool_contact(3);
                self.leg_contact_force_polytope(index(4),:,4) .* self.bool_contact(4);
                (self.asc_wrench_matrix(1:3,1) * self.asc_tension_space(n_asc_tension,1))';
                (self.asc_wrench_matrix(1:3,2) * self.asc_tension_space(n_asc_tension,2))'
            ];

            % Add inertia forces due to base acceleration
            inertia_forces = self.model.totalmass * self.ddot_p_base; % F = ma
            forces = [forces; inertia_forces'; self.com_force.b_norm'];

            % Calculate moments and forces
            for i = 1:8  % Updated to 8 to include inertia forces
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
                self.zmp(2, num_point) = self.total_moment(num_point, 1) / self.total_force(num_point, 3); % ZMP y-coordinate
                self.zmp(3, num_point) = 0; % ZMP z-coordinate (xy-plane)
            else
                % Handle case where the vertical force is zero to avoid division by zero
                self.zmp(:, num_point) = [NaN; NaN; NaN];
            end
        end
        return;
    end

    % Recursive case: Generate for loop only if c_bool(depth) is 1
    if self.bool_contact(depth) == 1
        for n = 1:4
            next_num_point = num_point + (n-1) * 4^(self.num_contact - depth);
            recursive_for(self, depth + 1, [index, n], next_num_point);
        end
    else
        % If c_bool(depth) is 0, skip this loop and move to the next depth
        recursive_for(self, depth + 1, [index, 1], num_point);
    end
end



