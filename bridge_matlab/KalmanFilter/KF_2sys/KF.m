function [x] = KF(z, dt)

persistent Q R
persistent x u P
persistent firstRun

m = 10;
b = 3;
k = 5;
a = 2;
w = 4;

if isempty(firstRun)
  Q = [ 0.01  0      0;
        0  0.01  0;
        0  0      0.01 ];
     
  R = 100*eye(2);

  x = [0 0]';  
  u = 0;
  P = 100*eye(2);
  
  firstRun = 1;  
end

A = [0 1; -k/m -b/m];
B = [0;1/m];
H = [1 0];

x = A*x + B*u;
P = A*P*A' + Q;

K = P*H'*inv(H*P*H'+R);

x = x + K*(z-H*x);




    
end