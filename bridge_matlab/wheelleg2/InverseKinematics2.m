function [q] = InverseKinematics2(p,self)
    lb = 257.6;
    l1 = 63.6;
    l2 = 300;
    l3 = 310;
    db = 65;
    d1 = 70;
    d2 = 29;
    d3 = 66;
    n1 = [1; 1; -1; -1];
    n2 = [1; -1; 1; -1];
    n3 = [-1; 1; 1; -1];

    % base movement
    Rg_b = rotz(self.q(3))*roty(self.q(2))*rotx(self.q(1));
    for i = 1:4
        p.b_w{i} = inv(Rg_b)*(p.g_w{i} - self.p);
    end

    % Inverse Kinematics
    for i = 1:4
    k1 = sqrt((p.b_w{i}(1)-n1(i)*lb)^2+(p.b_w{i}(2)-n2(i)*db)^2);
    beta = (acos((l1+d2+d3)/k1));
    % alpha = atan2(abs(p.b_w{i}(1)-n1(i)*lb) , abs(p.b_w{i}(2)-n2(i)*db));
    % q.hr(i) = n2(i)*(beta-alpha);
    alpha = atan2(abs(p.b_w{i}(1)-n1(i)*lb) , abs(p.b_w{i}(2)-n2(i)*db));
    q.hr(i) = n3(i)*(beta-alpha);
    k2 = (l1+d2+d3)*tan(beta);

    k3 = sqrt(k2^2+(-p.b_w{i}(3)-d1)^2);
    gamma = (asin(((-p.b_w{i}(3)-d1))/k3));

    if sign(p.b_w{i}(1)-n1(i)*lb) <0
        gamma = pi- gamma;
    end

    q.k(i) = (acos((k3^2-l2^2-l3^2)/(2*l2*l3)));

%     if pi < q.k(i) &&  q.k(i) < 2*pi
%         q.k(i) = 2*pi - q.k(i);
%     end
    
    
    k4 = l3*cos((q.k(i)))+l2;
    k5 = l3*sin((q.k(i)));
    phi = atan2(k5,k4);


%     q.hp(i) = phi - (gamma - pi/2);
    q.hp(i) = phi + gamma - pi/2;
    end

end