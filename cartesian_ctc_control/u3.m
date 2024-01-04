function u_output = u3(u)
m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
q1 = u(1); q2 = u(2); q3 = u(3);

J11 = -sin(q1)*(l3*cos(q2+q3) +l2*cos(q2));
J12 = -cos(q1)*(l3*sin(q2+q3) +l2*sin(q2));
J13 = -cos(q1)*(l3*sin(q2+q3));
J21 = cos(q1)*(l3*cos(q2+q3) +l2*cos(q2));
J22 = -sin(q1)*(l3*sin(q2+q3) +l2*sin(q2));
J23 = -sin(q1)*l3*sin(q2+q3);
J31 = 0;
J32 = l2*cos(q2) +l3*cos(q2+q3);
J33 = l3*cos(q2+q3);

J = [J11 J12 J13;J21 J22 J23;J31 J32 J33];
Jinv = inv(J);
u_input = [u(4); u(5); u(6)];

u_output = Jinv*u_input; 
end

