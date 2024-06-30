function h = Coriolis3(u)
m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
dotq1 = u(1); dotq2 = u(2); dotq3 = u(3); q1 = u(4); q2 = u(5); q3 = u(6);
h1 = (-2/3*m2*l2^2*sin(q2)*cos(q2)-2/3*m3*l3^2*cos(q2+q3)*sin(q2+q3)-m3*l2*l3*(cos(q2)*sin(q2+q3)+sin(q2)*cos(q2+q3)) ...
    -2*m3*l2^2*sin(q2)*cos(q2))*dotq1*dotq2-(2/3*m3+l3^2*sin(q2+q3)*cos(q2+q3)+m3*l2*l3*cos(q2)*sin(q2+q3))*dotq1*dotq3;
h2 = (1/3*m3*l2^2*sin(q2)*cos(q2)+1/3*m3*l3^2*cos(q2+q3)*sin(q2+q3)+1/2*m3*l2*l3*(cos(q2)*sin(q2+q3)+sin(q2)*cos(q2+q3)) ...
    +m3*l2^2*sin(q2)*cos(q2))*dotq1^2 - m3*l2*l3*sin(q3)*dotq2*dotq3 - 1/2*m3*l2*l3*sin(q3)*dotq2^2;
h3 = (1/3*m3*l3^2*sin(q2+q3)*cos(q2+q3)+1/2*m3*l2*l3*cos(q2)*sin(q2+q3))*dotq1^2 +(1/2*m3*l2*l3*sin(q3))*dotq2^2;
h = [h1;h2;h3];
end