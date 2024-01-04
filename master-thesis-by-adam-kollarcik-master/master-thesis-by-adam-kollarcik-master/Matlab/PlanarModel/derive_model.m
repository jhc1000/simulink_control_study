clear all
parametersToWorkspace;
syms th0 th1 th2 th4 th3 dth0 dth1 dth2 dth3 dth4 dq0 dq1 dq2 dq3 dq4 v1 v2 v3 dv1 dv2 dv3 t kspring k_ang u0 u3 real 
syms q0(t) q1(t) q2(t) q3(t) q4(t)
v = [v1;v2;v3];
dv = [dv1;dv2;dv3];
th = [th0;th1;th2;th3;th4];
q = [q0;q1;q2;q3;q4];
dq = [dq0;dq1;dq2;dq3;dq4];
q_diffs = [diff(q0, t);diff(q1, t);diff(q2, t);diff(q3, t);diff(q4, t)];
dth = [dth0;dth1;dth2;dth3;dth4];
[x0,x1,x12,x2,x23,x14,x4,x34,x3,x45] = get_points(th0,th1,th2,th3,th4,r,r3,ang_3,0,0,l1c,l11,l2c,l4c,l3,a_body,b_body);

%%%constraints

% g1 = simplify(simplify(x34-x45));
g1 = [l2*cos(th2)+l3*cos(th2+th3)-l12+l11-l4*cos(th4);l2*sin(th2)+l3*sin(th2+th3)-l4*sin(th4)];
g1 = simplify(g1);
A = simplify(jacobian(g1,th));
A = subs(A,th,q);
dA = simplify(diff(A,t));
dA = subs(dA,q_diffs,dq);
g1 = g1==0;
G = simplify(null(A));
G = subs(G,th,q);
dG = simplify(diff(G,t));
% Cg = G'*D*dG


%%
x0 = simplify(x0);
x1 = simplify(x1);
x2 = simplify(x2);
x3 = simplify(x3);
x4 = simplify(x4);

x0 = subs(x0,th,q);
x1 = subs(x1,th,q);
x2 = subs(x2,th,q);
x3 = subs(x3,th,q);
x4 = subs(x4,th,q);

x0_dot = diff(x0,t);
x1_dot = diff(x1,t);
x2_dot = diff(x2,t);
x3_dot = diff(x3,t);
x4_dot = diff(x4,t);

% pendulum
xd = x3-x0;
ang0 = -(atan2(xd(2),xd(1))-pi/2);
ang0_dot = diff(ang0,t);
ang0_dot = simplify(subs(ang0_dot,q_diffs,dq));
ang0 = simplify(subs(ang0,q_diffs,dq));
%

x0_dot = simplify(subs(x0_dot,q_diffs,dq));
x1_dot = simplify(subs(x1_dot,q_diffs,dq));
x2_dot = simplify(subs(x2_dot,q_diffs,dq));
x3_dot = simplify(subs(x3_dot,q_diffs,dq));
x4_dot = simplify(subs(x4_dot,q_diffs,dq));

[Jv0,~] = equationsToMatrix(x0_dot,dq);
[Jv1,~] = equationsToMatrix(x1_dot,dq);
[Jv2,~] = equationsToMatrix(x2_dot,dq);
[Jv3,~] = equationsToMatrix(x3_dot,dq);
[Jv4,~] = equationsToMatrix(x4_dot,dq);




% w0 = [dq0];
% w1 = [dq1];
% w2 = [dq1+dq2];
% w4 = [dq1+dq4];
% w3 = [dq1+dq2+dq3];


Dw0 = diag([I0 0 0 0 0]);
Dw1 = diag([0 I1 0 0 0]);
Dw2 = [zeros(5,1) [zeros(1,3);[ones(2,2)*I2 zeros(2,1)]; zeros(2,3)] zeros(5,1)];
Dw3 = [zeros(5,1) [zeros(1,3); ones(3,3)*I3; zeros(1,3)] zeros(5,1)];
Dw4 = [zeros(5,1) [zeros(1,4); I4 0 0 I4 ; 0 0 0 0 ; 0 0 0 0; I4 0 0 I4]];



D = simplify(m0*(Jv0.'*Jv0) +  m1*(Jv1.'*Jv1) + m2*(Jv2.'*Jv2)  + m3*(Jv3.'*Jv3) + m4*(Jv4.'*Jv4)  + (Dw0+Dw1+Dw2+Dw3+Dw4));
P = x0(2)*m0*g +  x1(2)*m1*g + x2(2)*m2*g+ x3(2)*m3*g+ x4(2)*m4*g + 1/2*(th2-k_ang)^2*kspring ;


P = subs(P,q,th);
psi = jacobian(P,th).';
% psi = subs(psi,th,q);

 
% [Jw0,~] = equationsToMatrix(w0,dq);
% [Jw1,~] = equationsToMatrix(w1,dq);
% [Jw2,~] = equationsToMatrix(w2,dq);
% [Jw3,~] = equationsToMatrix(w3,dq);
% [Jw4,~] = equationsToMatrix(w4,dq);
D = subs(D,q,th);

LAMBDA = sym(zeros(5,5,5));
for i = 1:5
    for j = 1:5
        for k = 1:5
            LAMBDA(i,j,k) = 1/2*(diff(D(i,j),th(k)) + diff(D(i,k),th(j)) - diff(D(j,k),th(i)));
        end
    end
end
C = sym(zeros(5,5));
for i = 1:5
    for j = 1:5
        cij = 0;
        for k = 1:5
            cij = cij + LAMBDA(i,j,k)*dq(k);
        end
        C(i,j) = cij;
        
    end
end

% D = subs(D,th,q);
D = simplify(D);
ang0 = simplify(subs(ang0,[dq,q],[dth,th]));
ang0_dot = simplify(subs(ang0_dot,[dq,q],[dth,th]));
psi = simplify(subs(psi,q,th));
C = simplify(subs(C,dq,dth));
A = simplify(subs(A,[dq;q],[dth;th]));
dA = simplify(subs(dA,[dq;q],[dth;th]));
B = [1 0;1 0; 0 0 ; 0 1; 0 0];


%% no constraint DAE
% mass_M = [D zeros(5,5); zeros(5,5) eye(5)];
% f =  [D\(-C*dq-psi);dq];
% f =  [(-C*dth-psi);dth];

% mass_M = subs(mass_M,q,th);
% mass_M = subs(mass_M,dq,dth);
% f = subs(f,q,th);
% f = subs(f,dq,dth);


%% DAE model
syms lambda1 lambda2 real

mass_M = [D zeros(5,7);zeros(5,5) eye(5) zeros(5,2);A zeros(2,7)];
f = [(B*[u0;u3]-b*dth-C*dth-psi+A.'*[lambda1;lambda2]);dth;-dA*dth];

%% pseudo velocities

% -b*dth-C*dth-psi
G = simplify(subs(G,q,th));
dG =simplify(subs(dG,q_diffs,dth));
dG = simplify(subs(dG,q,th));
Mg = G.'*D*G;
Cg = G.'*D*dG+G.'*(C+b)*G;
Cg = simplify(subs(Cg,dth,G*v));

fq = G*v;
% tmp = -Cg*v - G.'*(b'.*(fq)) - G.'*psi+G.'*(B*[u0;u3]);

ang0_dot = subs(ang0_dot,dth,G*v);
fv = Mg\(-Cg*v - G.'*psi+G.'*(B*[u0;u3]));
fopt =   (- G.'*psi+G.'*(B*[u0;u3]));



%% Linearization
% load('eqpos.mat')
% f = [fv;fq];
% % +-
% th0_0 = 0;
% th1_0 = equilibrium(1);
% th2_0 = equilibrium(2);
% th3_0 = equilibrium(3);
% th4_0 = equilibrium(4);
% th_eq = [th0_0;th1_0;th2_0;th3_0;th4_0];
% 
% u0_0 = 0;
% u3_0 =  equilibrium(8);
% kspring_0 = equilibrium(5);
% kang_0 = equilibrium(6);
% 
% 
% 
% A_s = double(subs(jacobian(f, [v;th]),[v;th;u0;u3;kspring;k_ang],[0;0;0;th_eq;u0_0;u3_0;kspring_0;kang_0]));
% B_s = double(subs(jacobian(f, [u0;u3]),[v;th;u0;u3;kspring;k_ang],[0;0;0;th_eq;u0_0;u3_0;kspring_0;kang_0]));


% rank(ctrb(A_s,B_s))
% C_s = eye(8);
% [Aa_bar, Ba_bar, Ca_bar, T, k] = ctrbf(A_s, B_s, C_s);
% Aa_uc = Aa_bar(1:2, 1:2);
% Aa_c = Aa_bar(3:end, 3:end);
% Ba_c = Ba_bar(3:end,:);
% % 
% K_lqr = lqr(A_s,B_s,eye(8),eye(2),[])


% A_s_new =zeros(6,6);
% A_s_new(1:5,1:3) = A_s(1:5,1:3);
% A_s_new(6,1:6) = A_s(8,1:6);
% A_s_new(:,5:6) = A_s(:,5:end)*[1 0 ; 0 0 ; 0 0; 0 0 ]

% sysC = ss(A_s,B_s,eye(8),[]);
% sysD = c2d(sysC,0.05);
% Ad = sysD.A

%% save reduced ODE model & equilibirum find function
matlabFunction([fv;fq],'File','ODE_fixed_torque','Vars',{t,[v1;v2;v3; th0; th1; th2; th3; th4],[kspring;k_ang],[u0;u3]},'Optimize',true,'Outputs',{'dydt'});

matlabFunction(([fopt;0;0]-[zeros(3,1);lhs(g1)]),'File','eq_search_fixed_torque','Vars',{[th1; th2; th3;th4;kspring;k_ang;u0;u3]});

disp('function save done')
%% save DAE model
% matlabFunction(f,'File','DAE_const_final','Vars',{t,[dth0; dth1; dth2; dth3; dth4; th0; th1; th2; th3; th4;lambda1;lambda2 ],[kspring;k_ang],[u0;u3]});
% matlabFunction(mass_M,'File','mass_M_final','Vars',{t,[dth0; dth1; dth2; dth3; dth4; th0; th1; th2; th3;th4;lambda1;lambda2]});
