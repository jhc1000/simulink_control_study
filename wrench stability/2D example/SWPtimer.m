function SWPtimer(~, ~, handles)
global self

%% Stable Region
self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

self.slope = [0.0, deg2rad(20), 0.0];
% self.bool_contact = [1,1,1,1];

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
self.mu = 0.7;               % Friction coefficient

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

% Constraints: Aeq * x = beq
Aeq = [A1, A2, A3];
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
    % self.com_position_lp = [x(1); x(2); self.p_base(3)+max(self.p.b_w(3,:))];
    % self.grf = x(3:end);
    self.grf = x(3:end-2);
    self.tension_lp = x(end-1:end);

    % Save results to arrays
    self.com_position_lp_results(:, k) = self.com_position_lp;
    self.grf_results{k} = self.grf;

end
self.com_position_togo = mean(self.com_position_lp_results,2) - self.com_xy_position';

%% closest point of stable region
Vqp = self.com_position_lp_results; % Polytope vertex들
robot_pos = self.p_base; % 로봇의 위치

% 모든 점들과 로봇 위치 간의 거리 계산
distances = sqrt(sum((Vqp - robot_pos).^2, 1));

% 최소 거리를 가지는 점의 인덱스 찾기
[~, min_idx] = min(distances);

% 가장 가까운 점
self.x_qp_full = Vqp(:, min_idx);

% 로봇이 이동해야 할 위치 계산
% self.com_position_togo = self.x_qp_full - [self.p_base(1); self.p_base(2); 0.0];
% self.com_position_togo = self.x_qp_full - self.com_xy_position';

%% Topic Publish
t = ros2time(self.node,"now");

% Update the pose message values
if isvalid(handles.swpPub)
    handles.swpPubmsg.header.frame_id = 'base_link';
    handles.swpPubmsg.header.stamp = t;
    handles.swpPubmsg.point.x = self.com_position_togo(1);
    handles.swpPubmsg.point.y = self.com_position_togo(2);
    handles.swpPubmsg.point.z = 0.0;

    % Publish pose message
    send(handles.swpPub,handles.swpPubmsg);
end

% Update the com message values
if isvalid(handles.comPub)
    handles.comPubmsg.header.frame_id = 'base_link';
    handles.comPubmsg.header.stamp = t;
    handles.comPubmsg.point.x = self.com_xy_position(1)-self.p_base(1);
    handles.comPubmsg.point.y = self.com_xy_position(2)-self.p_base(2);
    handles.comPubmsg.point.z = self.com_xy_position(3)-self.p_base(3);

            
    % Publish pose message
    send(handles.comPub,handles.comPubmsg);
end

% Update the polytope message values
if isvalid(handles.swpolytopePub)
    handles.swpolytopePubmsg.header.frame_id = 'odom';
    handles.swpolytopePubmsg.header.stamp = t;
    for i = 1:width(self.com_position_lp_results)
        handles.swpolytopePubmsg.polygon.points(i).x = single(self.com_position_lp_results(1,i));
        handles.swpolytopePubmsg.polygon.points(i).y = single(self.com_position_lp_results(2,i));
        handles.swpolytopePubmsg.polygon.points(i).z = single(self.com_position_lp_results(3,i));
    end

    % Publish polytope message
    send(handles.swpolytopePub,handles.swpolytopePubmsg);
end


end