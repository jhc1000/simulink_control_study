function G = Gravity3(u)
m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
g = 9.8;
q1 = u(1); q2 = u(2); q3 = u(3);
G1 = 0;
G2 = 1/2*m2*l2*g*cos(q2) + m3*g*(1/2*l3*cos(q2+q3)+l2*cos(q2));
G3 = 1/2*m3*l3*g*cos(q2+q3);
G = [G1;G2;G3];
end