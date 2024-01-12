function [p] = ForwardKinematics2(q,self)
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

    % Base Coordinate     
    Rg_b = rotz(self.q(3))*roty(self.q(2))*rotx(self.q(1));
    T.g_b = TransformationMatrix(Rg_b,self.p);

    % define Rotation matrix
    for i = 1:4
        R.b_hr{i} = [0 0 1;1 0 0;0 1 0]*rotz(q.hr(i));
        R.hr_hp{i} = [0 0 1;-1 0 0;0 -1 0]*rotz(q.hp(i));
        R.hp_k{i} = [1 0 0;0 -1 0;0 0 -1]*rotz(q.k(i));
        R.k_w{i} = [-1 0 0;0 1 0;0 0 -1];
        
        p.b_hr{i} = [n1(i)*lb; n2(i)*db; 0];
        p.hr_hp{i} = [n2(i)*d1; 0; n1(i)*l1];
        p.hp_k{i} = [l2; 0; n2(i)*d2];
        p.k_w{i} = [l3; 0; -n2(i)*d3];
        
        % Forward Kinematics
        T.b_hr{i} = TransformationMatrix(R.b_hr{i},p.b_hr{i});
        T.hr_hp{i} = TransformationMatrix(R.hr_hp{i},p.hr_hp{i});
        T.hp_k{i} = TransformationMatrix(R.hp_k{i},p.hp_k{i});
        T.k_w{i} = TransformationMatrix(R.k_w{i},p.k_w{i});
        
        T.b_w{i} = T.b_hr{i}*T.hr_hp{i}*T.hp_k{i}*T.k_w{i};
        p.b_w{i} = T.b_w{i}([1,2,3],[4]);
        
        T.b_hp{i} = T.b_hr{i}*T.hr_hp{i};
        p.b_hp{i} = T.b_hp{i}([1,2,3],[4]);
        
        T.b_k{i} = T.b_hr{i}*T.hr_hp{i}*T.hp_k{i};
        p.b_k{i} = T.b_k{i}([1,2,3],[4]);

        % Global Coordinate
        T.g_hr{i} = T.g_b*T.b_hr{i};
        p.g_hr{i} = T.g_hr{i}([1,2,3],[4]);
        
        T.g_w{i} = T.g_b*T.b_w{i};
        p.g_w{i} = T.g_w{i}([1,2,3],[4]);
        
        T.g_hp{i} = T.g_b*T.b_hp{i};
        p.g_hp{i} = T.g_hp{i}([1,2,3],[4]);
        
        T.g_k{i} = T.g_b*T.b_k{i};
        p.g_k{i} = T.g_k{i}([1,2,3],[4]);

    end
end