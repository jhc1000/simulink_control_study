function [J] = Jacobian(p,q)
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
    
    for i=1:4
    A2{i} = (cos(q.hr(i))*(l2*cos(q.hp(i)+l3*cos(q.hp(i)-q.l(i))))-n2(i)*(d1+d2+d3)*sin(q.hr(i)));
    A3{i} = (sin(q.hr(i))*(l2*cos(q.hp(i))+l3*cos(q.hp(i)-q.k(i)))+n2(i)*(d1+d2+d3)*cos(q.hr(i)));
    B1{i} = -(l2*cos(q.hp(i))+l3*cos(q.hp(i)-q.k(i)));
    B2{i} = -sin(q.hr(i))*(l2*sin(q.hp(i))+l3*sin(q.hp(i)-q.k(i)));
    B3{i} = cos(q.hr(i))*(l2.sin(q.hp(i))+l3*sin(q.hp(i)-q.k(i)));
    C1{i} = l2*cos(q.hp(i));
    C2{i} = sin(q.hr(i))*l3*sin(q.hp(i)-q.k(i));
    C3{i} = -cos(q.hr(i))*l3*sin(q.hp(i)-q.k(i));

    J_b = [1, -1, p.b_w{i}(1)-p.b_w{i}(2);
           1,  1, -p.b_w{i}(1)-p.b_w{i}(2);
           1,  1, -p.b_w{i}(1)-p.b_w{i}(2);
           1, -1, p.b_w{i}(1)-p.b_w{i}(2)]

    J_
    end
end