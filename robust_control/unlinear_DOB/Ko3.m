function r = Ko3(u)
n = 3;
m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
ko = [100; 200; 300];
Ko = diag(ko);
u_h = [u(1);u(2);u(3)];
r = Ko*u_h;
end