% clear all
load('eq.mat')


%%


dth0_0 = 0;
dth1_0 =0 ;
dth2_0 = 0;
dth3_0 = 0;
dth4_0 = 0;
dth_0 =[0; 0; dth0_0; dth1_0; dth2_0; dth3_0; dth4_0];
th0_0 = 0;
load('eq.mat')
th1_0 = equilibrium(1);
th2_0 = equilibrium(2);
th3_0 = equilibrium(3);
th4_0 = equilibrium(4);

k_spring = equilibrium(5);
kang = equilibrium(6);
u0s = 0;
u3s =  equilibrium(8);


Xeq = [ 0;0;0;0;0;0;0;0;th1_0;th2_0;th3_0;th4_0];

%%
N = 500;    % Number of knots
Tf = 0.5;    % Final time
Ts = Tf/N; % Sampling time

N_liftoff = 300;
% Initialize the optimization problem
opti = casadi.Opti();

% define decision variables
X = opti.variable(12,N+1); % state trajectory
Dxw = X(1, :);
Dyw =X(2, :);
Dth0 = X(3, :);
Dth1 = X(4, :);
v = X(5, :);
xw = X(6, :);
yw = X(7, :);
th0 = X(8, :);
th1 = X(9, :);
th2 = X(10, :);
th3 = X(11, :);
th4 = X(12, :);
U = opti.variable(2,N+1); % control trajectory
L = opti.variable(2,N+1);% constraining forces
% Dynamic constraints
% ts_params = [0;0];  %torsion spring params
% ts_params = prms.ts;
% f = @(x,u,l) doubleLegSegway_constrForces_ode(x, u, ts_params, l); % dx/dt = f(x,u,l)
% 

% ---- Forward Euler --------
cost = 0;
for k=1:N % loop over control intervals
%     [M_next,C_next,N_next,B_next,J_next] = doubleLegSegway_manEq_matrices(X(:,k+1),ts_params);
    [M_next,C_next,N_next,B_next,J_next] = get_matrices_LIGHT(X(:,k+1),[k_spring;kang]);
    
    qk = X(6:end,k);
    Dqk = X(1:5,k);
    qk_next = X(6:end,k+1);
    Dqk_next = X(1:5,k+1);
    Dqk_un_next = pseudo_convert(X(:,k+1))*Dqk_next;
    
    opti.subject_to( qk - qk_next + Ts*Dqk_un_next == 0);
    opti.subject_to( M_next*(Dqk_next - Dqk) + Ts*(C_next*Dqk_next + N_next - B_next*U(:,k+1) - J_next'*L(:,k+1)) == 0);
    
%     cost = cost + Ts*U(:,k+1)'*U(:,k+1) + 0*(U(:,k+1)-U(:,k))'*(U(:,k+1)-U(:,k)); 
    cost = cost + 10*Ts*U(:,k+1)'*diag([1 1])*U(:,k+1) ; 
%     cost = cost + Ts*U(:,k+1)'*U(:,k+1)  ;
%      cost = 1;

end


% Objective function
opti.minimize( cost ); % minimize time
%% Initial constraints
% Start with both links handing down
% opti.subject_to(th1(1)  == x_stat(7));
opti.subject_to(th1(1)  == equilibrium(1));
opti.subject_to(Dth1(1) == 0);
% opti.subject_to(th2(1)  == x_stat(9));
opti.subject_to(th2(1)  == equilibrium(2));
opti.subject_to(th3(1)  == equilibrium(3));
opti.subject_to(th4(1)  == equilibrium(4));
opti.subject_to(v(1) == 0);

% Start with zero velocity and at the origin
opti.subject_to(th0(1)  == 0);
opti.subject_to(Dth1(1) == 0);
opti.subject_to(xw(1)  == 0);
opti.subject_to(Dxw(1)  == 0);
opti.subject_to(Dyw(1) == 0);
opti.subject_to(yw(1) == 0.07);

%% Stay on the floor constraint
% for k=1:N+1
%     opti.subject_to( ground_const(X(:,k)) == 0);
%     opti.subject_to( L(2,k) >= 0);    
% end


for k=1:N_liftoff
       opti.subject_to( ([1, 0, -0.07,0,0]*X(1:5,k)) == 0);
    opti.subject_to( (yw(k)-0.07) == 0); 
    opti.subject_to( L(2,k) >= 0);    
end

 for k=N_liftoff+1:N+1
    opti.subject_to( yw(k) > 0.07);   
    opti.subject_to( L(:,k) == 0); 

%     opti.subject_to( U(:,k) == 0);  
 end
 
%   for k=51:N+1
%     opti.subject_to( ([1, 0, -0.07,0,0]*X(1:5,k)) == 0);
%     opti.subject_to( (yw(k)-0.07) == 0); 
%    opti.subject_to( L(2,k) >= 0);   
% %     opti.subject_to( U(:,k) == 0);  
%  end


% 
%  for k=1:N+1    
%  opti.subject_to( ([1, 0, -0.07,0,0]*X(1:5,k))*L(1,k) == 0);
% %  opti.subject_to( (yw(k)-0.07)*L(1,k)  == 0);
%     opti.subject_to( (yw(k)-0.07) >= 0);   
%     opti.subject_to( (yw(k)-0.07)*L(2,k) == 0); 
%     opti.subject_to( L(2,k) >= 0);   
% end


% opti.subject_to(y0(1) == prms.r);
% opti.subject_to(x0(1) == prms.r*th0(1));

%% Limit the torque
opti.subject_to(-2 <= U(1,:) <= 2)
opti.subject_to(-10 <= U(2,:) <= 10)
%% Final-time constraints
% End with both links handing down
opti.subject_to( -deg2rad(30) < (X(9:end,:) - Xeq(9:end)) < deg2rad(30));
% opti.subject_to(Dth1(end) == 0);
% opti.subject_to(v(end) == 0);
% opti.subject_to(Dth0(end) == 0);
% opti.subject_to(Dth1(end) == 0);
% opti.subject_to(Dxw(end) == 0);
% opti.subject_to(Dyw(end) == 0);
opti.subject_to(xw(end) <= -0.4);
% opti.subject_to(Dth0(end) == 0);
opti.subject_to(Dth1(end) == 0);
opti.subject_to(deg2rad(5) <= th1(end) + th2(end) + th3(end) -th2_0 -th3_0 -th1_0 <= deg2rad(10) );
opti.subject_to(deg2rad(0)  <= th1(end)  -th1_0);
opti.subject_to(v(end) == 0);
% opti.subject_to(yw(end) == .1);
% opti.subject_to(deg2rad(10
% Ned with zero velocity and at the position one motor away from the origin
%opti.subject_to(yw(200)  == 0.7);
% opti.subject_to( yw(350)  == 0.27);
opti.subject_to( yw(end)  == 0.07);
% opti.subject_to( yw(end)  == 0.07);
% opti.subject_to(Dxw(end) <= -0.3);

% Initialize decision variables
opti.set_initial(U, 0.01*randn(2,N+1));
opti.set_initial(L(2,:), 9.81*2.3);
opti.set_initial(L(1,:), randn(1,N+1));
opti.set_initial(X, repmat(Xeq,1,N+1));

% Solve NLP
p_opts = struct();
s_opts = struct('max_iter', 4e4);
opti.solver('ipopt', p_opts, s_opts); % set numerical backend
sol = opti.solve();   % actual solve
%%
% Extract the states and control from the decision variables
x_star = sol.value(X)';
u_star = sol.value(U)';
l_star = sol.value(L)';

 t = 0:Ts:Ts*N;

figure(1)
clf

subplot(411)
stairs(t, x_star(:,1:5))
legend('dxw','dyw','dth0','dth1','dth4')
grid on

subplot(412)
stairs(t, x_star(:,6:end))
legend('xw','yw','th0','th1','th2','th3','th4')
grid on
subplot(413)
stairs(t(2:end), u_star(2:end,:))
legend('u0','u3')
grid on
subplot(414)
stairs(t(2:end), l_star(2:end,:))
legend('l1','l2')
grid on
%%
parametersToWorkspace
params = [r,r3,ang_3,0,0,l1c,l11,l2c,l4c,l3,a_body,b_body];%  y =[zeros(6,1);x(1:4

record = false;%% true for recording video, add vargargin to specify the filename
configOnly = false; %% true for showing only the initial configuration
xrange = [-2 2]; %% xlims9 
yrange = [-0.1 0.6];%% ylims
secondMonitor = true; %% display animation on second monitor

 drawStoupento2D(t,x_star,params,xrange,yrange,configOnly,record,secondMonitor)
 
 %%
 n = size(u_star,1);
 mult = 1;
 u0jump = zeros(n* mult,1);
 u3jump = zeros(n* mult,1);
 for idx = 1:n
    u0jump((idx-1)* mult +1: (idx)* mult) = u_star(idx,1);
    u3jump((idx-1)* mult +1: (idx)* mult) = u_star(idx,2);
     
 end
 
 
 
 
 %%
%  actual_vel = zeros(7,501);
%  for idx = 1 : 501
%  actual_vel(:,idx) =  pseudo_convert(x_star(idx,:)')*x_star(idx,1:5)';
%  end
%  
%  figure
%  stairs(t,actual_vel')
%   figure
%  stairs(t, x_star(:,1:5))
%  
%  kycel_vel = actual_vel(6,:)';
%  kycel_ang = x_star(:,11);
%  stairs(1:n*10,u0jump)

% %%
% % waitforbuttonpress;                  
% visu(t, x_star, prms, 200, 8, 'jump.gif')
% %%
% x_0 = x_stat;
% Tf = t(end);
% tau_opt = timeseries(u_star(2:end,:), t(2:end))