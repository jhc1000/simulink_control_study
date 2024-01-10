close all;
clear all;
clc;
%%
n = 3;
Ac1 = zeros(n); 
Ac2 = -eye(n); 
Ac3 = zeros(n);
Ac4 = zeros(n);
Ac = [Ac1 Ac2; Ac3 Ac4]

Cc = [eye(3) zeros(3)]
v = 0.001*ones(n,1);
Rc = diag(v)
w = 0.1*[zeros(n,1); ones(n,1)];
Qc = diag(w)

%   X = Riccati(A,G,Q) solves the algebraic Riccati equation of the form:
%       A'*X + X*A' - X*G*X + Q = 0, where X is symmetric.

A = Ac'
Rcinv = Rc^(-1)
G = Cc'*Rcinv*Cc
Q = Qc
P = Riccati(A,G,Q)

L = P*Cc'*Rc^(-1)