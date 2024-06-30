function Jdotqdot = Jdotqdot3(u)
m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
q1 = u(1); q2 = u(2); q3 = u(3); qdot1 = u(4); qdot2 = u(5); qdot3 = u(6);

Jd11 = -cos(q1)*qdot1*(l3*cos(q2+q3) +l2*cos(q2)) -sin(q1)*(-l3*sin(q2+q3)*(qdot2+qdot3) -l2*sin(q2)*qdot2);
Jd12 = -sin(q1)*qdot1*(l3*sin(q2+q3) +l2*sin(q2)) -cos(q1)*(l3*cos(q2+q3)*(qdot2+qdot3) +l2*cos(q2)*qdot2);
Jd13 = sin(q1)*qdot1*(l3*sin(q2+q3)) -cos(q1)*(l3*cos(q2+q3)*(qdot2+qdot3));
Jd21 = -sin(q1)*qdot1*(l3*cos(q2+q3) +l2*cos(q2)) +cos(q1)*(-l3*sin(q2+q3)*(qdot2+qdot3) -l2*sin(q2)*qdot2);
Jd22 = -cos(q1)*qdot1*(l3*sin(q2+q3) +l2*sin(q2)) -sin(q1)*(l3*cos(q2+q3)*(qdot2+qdot3) +l2*cos(q2)*qdot2);
Jd23 = -cos(q1)*qdot1*l3*sin(q2+q3) -sin(q1)*l3*cos(q2+q3)*(qdot2+qdot3);
Jd31 = 0;
Jd32 = -l2*sin(q2)*qdot2 -l3*sin(q2+q3)*(qdot2+qdot3);
Jd33 = -l3*sin(q2+q3)*(qdot2+qdot3);

Jdot = [Jd11 Jd12 Jd13;Jd21 Jd22 Jd23;Jd31 Jd32 Jd33];

qdot = [qdot1; qdot2; qdot3];
Jdotqdot = Jdot*qdot;
end