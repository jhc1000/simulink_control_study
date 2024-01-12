function [pos, vel] = DvKalman(z,dt)

persistent A H Q R 
persistent x P 
persistent firstrun

if isempty(firstrun)
    %system equation & noise
    A = [1 dt;
        0 1];

    H = [1 0];

    Q = [1 0; 0 3];
    
    R = 10;

    x = [0 20]';
    P = 5*eye(2);

    firstrun = 1;
end

xp = A*x;
Pp = A*P*A' + Q;
K = Pp*H'/(H*Pp*H' + R);
x = xp + K*(z - H*xp);
P = Pp - K*H*Pp;

pos = x(1);
vel = x(2);

end