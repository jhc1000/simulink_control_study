function tr = Cartesian_trajectory(u)
l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
x0 = 0.408; y0 = 0.408; r = 0.2;

i = u;

% Circular trajectory
    yd = y0 +cos(-pi/6)*(r*cos(i));
    xd = x0 +r*sin(i);
    zd = l1-sin(-pi/6)*(r*cos(i));

% Inverse kinematics
    d = sqrt(xd^2+yd^2+(zd-l1)^2);
    a = (d^2-l2^2-l3^2)/(2*l2*l3);
    qd3 = atan2(-sqrt(1-a^2), a);
    alpha = atan2(zd-l1, sqrt(xd^2+yd^2));
    E = (l2^2+d^2-l3^2)/(2*l2*d);
    beta = atan2(sqrt(1-E^2), E);
    qd2 = alpha + beta;
    qd1 = atan2(yd, xd);

tr = [qd1, qd2, qd3, xd, yd, zd];
end

