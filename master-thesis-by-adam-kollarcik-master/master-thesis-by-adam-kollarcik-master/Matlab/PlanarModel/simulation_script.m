%% params

%% equilibirium init. cond.
dth0_0 = 0;
dth1_0 =0 ;
dth2_0 = 0;
dth3_0 = 0;
dth4_0 = 0;
dth_0 =[dth0_0; dth1_0; dth2_0; dth3_0; dth4_0];
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


%% ODE simulation
load('pendulumCTR.mat');

  

tspan = 0:0.01:8;
y0 = [0;0;0 ; th0_0; th1_0 - deg2rad(0); th2_0; th3_0; th4_0];
 options = odeset('OutputFcn',@odeprog,'Events',@odeabort,'RelTol',1e-12);
[t,y] = ode15s(@(t,y)ODE_fixed_torque(t,y,[k_spring;kang],stoupentoController(t,y,K,100,-150,equilibrium,ang00)),tspan,y0,options );
% %   [t,y] = ode15s(@(t,y)ODE_fixed_damping(t,y,[k_spring;kang],[0;0]),tspan,y0,options );
  %% DAE simulation 


%% plot states & inputs in time
plotAng = true;
plotVel = true;
plotIn = true;
ang_xrange = [t(1) t(end)];
ang_yrange = [-180 150];
vel_xrange = [t(1) t(end)];
vel_yrange = [-3.5*pi 3.5*pi];
in_xrange = [t(1) t(end)];
in_yrange = [-10 10];
plotStates(t,y,ang_xrange,ang_yrange,vel_xrange,vel_yrange,in_xrange,in_yrange,@(t,y)stoupentoController(t,y,K,100,-150,equilibrium,ang00),equilibrium,plotAng,plotVel,plotIn)
%% robot animation
parametersToWorkspace
params = [r,r3,ang_3,0,0,l1c,l11,l2c,l4c,l3,a_body,b_body];%  y =[zeros(6,1);x(1:4

record = false;%% true for recording video, add vargargin to specify the filename
configOnly = false; %% true for showing only the initial configuration
xrange = [-0.2 0.2]; %% xlims9 
yrange = [-0.1 0.6];%% ylims
secondMonitor = false; %% display animation on second monitor

 drawStoupento2D(t,y,params,xrange,yrange,configOnly,record,secondMonitor)
% drawStoupento2D(t,y(:,3:end),params,xrange,yrange,configOnly,record)%DAE animation
 
