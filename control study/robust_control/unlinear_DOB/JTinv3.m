function JTinv_u = JTinv3(u)
l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
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
JT = J';
JT_pseudo = (J*JT)^(-1)*J;
u_input = [u(4); u(5); u(6)];
JTinv_u = JT_pseudo*u_input;
end

