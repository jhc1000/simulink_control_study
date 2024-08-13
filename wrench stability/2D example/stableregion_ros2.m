close all;
clear all;
clc;

global self
% 전체 경로 추가
context();

self.robot_name = "wlar_2d";

self.model = wlar_model;
self.kinematics = wlar_kinematics;
self.dynamics = dynamics;

self = self.model.init(self);
self = self.kinematics.init(self);
self = self.dynamics.init(self);

self.slope = [0.0, deg2rad(0), 0.0];

self.anchor.position(:,:,1) = [0.0; 0.0; 0.250];
self.anchor.position(:,:,2) = [0.0; -3.944; 0.250];

self.q_base = [0.0; deg2rad(0); 0.0];
% self.p_base = [-2.0; -1.972; 0.50322];
self.p_base = [-2.0; -1.972; 0.50322];
% self.p_base = [-2.0; -1.972; 0.38545];
% self.p_base = [-1.0; -3.0; 0.50322];
% self.p_base = [-4.0; -2.5; 0.50322];
% self.p_base = [-2.0; -1.0; 0.50322];

self.base_movement = [-0.0; 0.0; 0.0];
% self.base_movement = [-0.25703; -0.11508; 0.0]; % lf
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

%% Contact Wrench Cone
self = geometry_computation.compute_contact_wrench_polytope(self);

%% Ascender Tension Wrench Polytope
self = geometry_computation.compute_ascender_wrench_polytope(self);

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
f_obj = [-ai; -ones(num_f, 1); ones(num_t,1)];
% f_obj = [-ai; -ones(num_f, 1)];

% Constraints: Aeq * x = beq
Aeq = [A1, A2, A3];
% Aeq = [A1, A2];
beq = u;

% Inequality constraints: A * x <= b
A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2)+size(W,2));
    zeros(size(B,1),size(cxy,1)),B,zeros(size(B,1),size(W,2));
    zeros(size(G,1),size(cxy,1)),G,zeros(size(G,1),size(W,2));
    zeros(size(W,1),size(cxy,1)),zeros(size(W,1),size(G,2)),W;
    zeros(size(H,1),size(cxy,1)),zeros(size(H,1),size(G,2)),H];

b_ineq = [zeros(size(cxy,1),1);
    zeros(size(B, 1), 1);
    d;
    w;
    h];


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

% 
% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(G,2));
%     zeros(size(G,1),size(cxy,1)),G];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     d];

% A_ineq = [zeros(size(cxy,1),size(cxy,1)+size(B,2));
%     zeros(size(B,1),size(cxy,1)),B];
% 
% b_ineq = [zeros(size(cxy,1),1);
%     zeros(size(B, 1), 1)];

lb = [];
ub = [];


% Define different values for ai
ai_values = [1,0; sqrt(1/2),sqrt(1/2); 0,1; -sqrt(1/2),sqrt(1/2); -1,0; -sqrt(1/2),-sqrt(1/2); 0,-1; sqrt(1/2),-sqrt(1/2)];

% Initialize arrays to store the results
com_position_lp_results = zeros(3, length(ai_values));
grf_results = cell(1, length(ai_values));

% 옵션 설정: 출력 안 함
options = optimoptions('linprog', 'Display', 'none');

% Loop over each ai value
for k = 1:size(ai_values, 1)
    ai = ai_values(k, :)';
    
    % Objective function: -ai.' * cxy (maximize ai.' * cxy -> minimize -ai.' * cxy)
    % f_obj = [-ai; -ones(num_f, 1)];
    f_obj = [-ai; -ones(num_f, 1); ones(num_t,1)];
    
    % Solve the linear program
    [x, fval, exitflag, output] = linprog(f_obj, A_ineq, b_ineq, Aeq, beq, lb, ub, options);
    
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

%% Plotting
plotting_tools.plot_robot_space(self);

%% ROS2
disp("Ros2 node start");
setenv("ROS_DOMAIN_ID","13")
self.node = ros2node("/stable_region",13);

swpPub = ros2publisher(self.node,'/pose_swp','geometry_msgs/PoseStamped');
swpPubmsg = ros2message(swpPub);
swpolytopePub = ros2publisher(self.node,'/polytope_swp','geometry_msgs/PolygonStamped');
swpolytopePubmsg = ros2message(swpolytopePub);
pause(3); %wait for some time to register publisher on the network
wheelegJointSub = ros2subscriber(self.node,'/wheelleg_joint_state_desired',@wheelleg_joint_state_callback);
RobotPositionSub = ros2subscriber(self.node,'/robot_position',@robot_position_callback);
global joint_state
global current_robot_position

timerHandles.swpPub = swpPub;
timerHandles.swpPubmsg = swpPubmsg;
timerHandles.swpolytopePub = swpolytopePub;
timerHandles.swpolytopePubmsg = swpolytopePubmsg;

simTimer = ExampleHelperROSTimer(0.01, {@SWPtimer,timerHandles});