d = 0.29;
l = 0.2907;
r = 0.07;
mb = 4;
mw = 0.1;
J = 0.000735;
K = 0.00039;
I1 = 0.015625;
I2 = 0.0118625;
I3 = 0.0118625;
calpha = 0.01;
g = 9.81;

syms x th psi dx dth dpsi uL uR



a11 = mb + 2*mw + 2*J/r^2;
a12 = mb*l*cos(th);
a21 = a12;
a22 = I2 + mb*l^2;
a33 = I3 + 2*K+(mw+J/r^2)*d^2/2 - (I3-I1-mb*l^2)*sin(th)^2;
c12 = -mb*l*dth*sin(th);
c13 = -mb*l*dpsi*sin(th);
c23 = (I3-I1-mb*l^2)*dpsi*sin(th)*cos(th);
c31 = mb*l*dpsi*sin(th);
c32 = -(I3-I1-mb*l^2)*dpsi*sin(th)*cos(th);
c33 = -(I3-I1-mb*l^2)*dth*sin(th)*cos(th);
d11 = 2*calpha/r^2;
d12= -2*calpha/r;
d21 = d12;
d22 = 2*calpha;
d33 = (d^2/(2*r^2))*calpha;

M = [a11 a12 0; a21 a22 0; 0 0 a33];
C = [0 c12 c13; 0 0 c23; c31 c32 c33];
D = [d11 d12 0; d21 d22 0; 0 0 d33];
B = [1/r 1/r; -1 -1; -d/(2*r) d/(2*r)];
G = [0 -mb*l*g*sin(th) 0].';


fv = M\(-(C+D)*[dx;dth;dpsi] - G + B*[-uL;-uR]);
f = [fv;[dx;dth;dpsi]];

Ac = double(subs(jacobian(f,[dx;dth;dpsi;x;th;psi]),[dx;dth;dpsi;x;th;psi;uL;uR],[0;0;0;0;0;0;0;0]));
Bc = double(subs(jacobian(f,[uL,uR]),[dx;dth;dpsi;x;th;psi;uL;uR],[0;0;0;0;0;0;0;0]));
sysfull_dicrete = c2d(ss(Ac,Bc,eye(6),[]),0.001,'zoh');
Ac_reduced = Ac([1:3,5],[1:3,5]);
Bc_reduced = Bc([1:3,5],:);
sysc = ss(Ac_reduced,Bc_reduced,eye(4),[]);
T = [0 0 -d/r 0; -2/r 0 0 0; 0 1 0 0; 0 0 0 1];
sysc = ss2ss(sysc,T);
sysd = c2d(sysc,0.001,'zoh');
Ad = sysd.A;
Bd = sysd.B;

%%
Aint = [Ad, zeros(4,2); -1 0 0 0 1 0; 0 -1 0 0 0 1];
Bint = [Bd;zeros(2,2)];
KpendI = dlqr(Aint,Bint,diag([15,5,1,1,1e-4,1e-4]),diag([1000,1000]),[])

Kpend = KpendI(:,1:end-2);
KI = KpendI(:,end-1:end); 
%

%% 
sysc = ss(Ac_reduced,Bc_reduced,eye(4),[]);
sys_dn = c2d(sysc,0.001,'zoh');
A_i = sys_dn.A;
B_i = sys_dn.B;
Tinv = inv([0 0 -d/r 0 0 0; -2/r 0 0 0 0 0; 0 1 0 0 0 0; 0 0 0 1 0 0 ; 0 0 0 0 -d/r 0; 0 0 0 0 0 -2/r]);
Aint = [A_i, zeros(4,2); 0 0 -1 0 1 0; -1 0 0 0 0 1];
Bint = [B_i;zeros(2,2)];
KpendII = dlqr(Aint,Bint,diag([816.3265*5,1, 20.8980*15,1,20.8980*1e-4,816.3265*1e-4]),diag([1000,1000]),[])./[-2/r, 1,-d/r ,1,-d/r,-2/r ]



