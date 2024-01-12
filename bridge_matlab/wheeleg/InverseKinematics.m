function [q] = InverseKinematics(p)
    lb = 257.6;
    l1 = 70;
    l2 = 300;
    l3 = 310;
    db = 65;
    d1 = 63.6;
    d2 = 29;
    d3 = 66;
    n1 = [1; 1; -1; -1];
    n2 = [1; -1; 1; -1];

    % Inverse Kinematics
    for i = 1:4
    k1 = sqrt((p.b_w{i}(2)-n2(i)*db)^2+(p.b_w{i}(3))^2);
    beta = acos((d1+d2+d3)/k1);
    alpha = atan2(-p.b_w{i}(3),n2(i)*p.b_w{i}(2)-db);
    q.hr(i) = n2(i)*(beta-alpha);
    
    k5 = (d1+d2+d3)*tan(beta);
    k4 = sqrt(k5^2+(p.b_w{i}(1)-n1(i)*(l1+lb))^2);
    q.k(i) = acos((k4^2-l2^2-l3^2)/(2*l2*l3));
    
    k2 = l3*cos(q.k(i))+l2;
    k3 = l3*sin(q.k(i));
    phi = atan2(p.b_w{i}(1)-n1(i)*(l1+lb),k5);
    lambda = atan2(k3,k2);
    q.hp(i) = lambda - phi;
    end

end