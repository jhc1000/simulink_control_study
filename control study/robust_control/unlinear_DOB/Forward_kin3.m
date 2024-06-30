function P = Forward_kin3(u)
l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
q1 = u(1); q2 = u(2); q3 = u(3);
Px = cos(q1)*(l3*cos(q2+q3) + l2*cos(q2));
Py = sin(q1)*(l3*cos(q2+q3) + l2*cos(q2));
Pz = l1 + l2*sin(q2) + l3*sin(q2+q3);

P = [Px;Py;Pz];
end

