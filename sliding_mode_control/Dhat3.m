function out = Dhat3(u)
m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
q1 = u(1); q2 = u(2); q3 = u(3);
alpha = 0.5;
d11 = 1/3*m2*l2^2*cos(q2)^2+1/3*m3*l3^2*(cos(q2+q3))^2+m3*l2*l3*cos(q2)*cos(q2+q3)+m3*l2^2*cos(q2)^2;
d12 = 0;
d13 = 0;
d21 = 0;
d22 = 1/3*m2*l2^2+1/3*m3*l3^2+m3*l2^2+m3*l2*l3*cos(q3);
d23 = 1/3*m3*l3^2+1/2*m3*l2*l3*cos(q3);
d31 = 0;
d32 = d23;
d33 = 1/3*m3*l3^2;
D = alpha*[d11 d12 d13; d21 d22 d23; d31 d32 d33];
u_h = [u(4);u(5);u(6)];
out = D*u_h;
end