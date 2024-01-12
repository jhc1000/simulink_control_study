function xc = KF(z)

persistent A H Q R 
persistent x P 
persistent firstrun

if isempty(firstrun)
    %system equation & noise
    A = 1;
    H = 0.5;
    Q = 0.0000 1;
    R = 1;

    x = 0;
    P = 1;

    firstrun = 1;
end

xp = A*x;
Pp = A*P*A' + Q;
K = Pp*H'*inv(H*Pp*H' + R);
x = xp + K*(z - H*xp);
P = Pp - K*H*Pp;

xc = x;
    
end

