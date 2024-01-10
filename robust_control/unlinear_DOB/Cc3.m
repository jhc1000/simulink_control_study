function out = Cc3(u)
n = 3;
m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
Cc = [eye(3) zeros(3)];
u_h = [u(1);u(2);u(3);u(4);u(5);u(6)];
out = Cc*u_h;
end