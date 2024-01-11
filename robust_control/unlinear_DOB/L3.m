function out = L3(u)
u_h = [u(1);u(2);u(3)];
% L =[4.0404    0.0000    0.0000;
%    -0.0000    4.0404    0.0000;
%     0.0000   -0.0000    4.0404;
%    -3.1623    0.0000    0.0000;
%     0.0000   -3.1623    0.0000;
%    -0.0000    0.0000   -3.1623];

n = 3;
Ac1 = zeros(n); 
Ac2 = -eye(n); 
Ac3 = zeros(n);
Ac4 = zeros(n);
Ac = [Ac1 Ac2; Ac3 Ac4];

Cc = [eye(3) zeros(3)];
v = 0.001*ones(n,1);
Rc = diag(v);
w = 0.01*[ones(n,1); ones(n,1)];
Qc = diag(w);

A = Ac';
Rcinv = Rc^(-1);
G = Cc'*Rcinv*Cc;
Q = Qc;
P = Riccati(A,G,Q);

L = P*Cc'*Rc^(-1);
out = L*u_h;
end