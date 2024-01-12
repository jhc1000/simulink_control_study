function [q] = InverseKinematics(p)
    l1 = 200;
    l2 = 200;
    

    % Inverse Kinematics
    for i = 1:1
    k1 = sqrt((p.b_c3{i}(1))^2+(p.b_c3{i}(2))^2);
    q.c2(i) = acos((k1^2-l1^2-l2^2)/(2*l1*l2));

    if q.c2(i) > pi
        q.c2(i) = 2*pi - q.c2(i);
    end

    k2 = l1 + l2*cos(q.c2(i));
    k3 = l2*sin(q.c2(i));
    alpha = atan2(k3,k2);
    beta = atan2(p.b_c3{i}(2),p.b_c3{i}(1));
    q.c1(i) = alpha + beta;
    end

end