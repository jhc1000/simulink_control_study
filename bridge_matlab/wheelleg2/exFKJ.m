close all
clear all
clc

%% function [p] = ForwardKinematics2(q,self)
    syms roll pitch yaw bx by bz qhr qhp qk qw lb l1 l2 l3 db d1 d2 d3 n1 n2 xg1 xg2 xg3 yg1 yg2 yg3 zg1 zg2 zg3
    self.p = [bx; by; bz];
    self.q = [roll; pitch; yaw];
    q = [qhr; qhp; qk; qw];

%     lb = 257.6;
%     l1 = 63.6;
%     l2 = 300;
%     l3 = 310;
%     db = 65;
%     d1 = 70;
%     d2 = 29;
%     d3 = 66;
%     n1 = [1; 1; -1; -1];
%     n2 = [1; -1; 1; -1];

    % Base Coordinate     
    R.g_b = rotz(self.q(3))*roty(self.q(2))*rotx(self.q(1));
    T.g_b = TransformationMatrix(R.g_b,self.p);

    % define Rotation matrix
    for i = 1:4
        R.b_hr{i} = [0 -1 0;1 0 0;0 0 1]*rotz(qhr);
        R.hr_hp{i} = [0 0 1;0 1 0;-1 0 0]*rotz(qhp);
        R.hp_k{i} = [1 0 0;0 -1 0;0 0 -1]*rotz(qk);
        R.k_w{i} = [-1 0 0;0 1 0;0 0 -1];
        
        p.b_hr{i} = [n1*lb; n2*db; 0];
        p.hr_hp{i} = [n2*l1; 0; -d1];
        p.hp_k{i} = [l2; 0; n2*d2];
        p.k_w{i} = [l3; 0; -n2*d3];

        p.hr_g1 = [xg1; yg1; zg1];
        p.hp_g2 = [xg2; yg2; zg2];
        p.k_g3 = [xg3; yg3; zg3];
        
        % Forward Kinematics
        T.b_hr{i} = TransformationMatrix(R.b_hr{i},p.b_hr{i});
        T.hr_hp{i} = TransformationMatrix(R.hr_hp{i},p.hr_hp{i});
        T.hp_k{i} = TransformationMatrix(R.hp_k{i},p.hp_k{i});
        T.k_w{i} = TransformationMatrix(R.k_w{i},p.k_w{i});

        T.hr_g1{i} = TransformationMatrix(eye(3),p.hr_g1);
        T.hp_g2{i} = TransformationMatrix(eye(3),p.hp_g2);
        T.k_g3{i} = TransformationMatrix(eye(3),p.k_g3);
        
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

        T.g_g1{i} = T.g_b*T.b_hr{i}*T.hr_g1;
        p.g_g1{i} = T.g_g1{i}([1,2,3],[4]);

        T.g_g2{i} = T.g_b*T.b_hr{i}*T.hr_hp{i}*T.hp_g2;
        p.g_g2{i} = T.g_g2{i}([1,2,3],[4]);

        T.g_g3{i} = T.g_b*T.b_hr{i}*T.hr_hp{i}*T.hp_k{i}*T.k_g3;
        p.g_g3{i} = T.g_g3{i}([1,2,3],[4]);

        %% Jacobian
        
        dx.g1{i} = diff(p.g_g1{i}, q(i));
        dx.g2{i} = diff(p.g_g2{i}, q(i));
        dx.g3{i} = diff(p.g_g3{i}, q(i));

        z0 = [0;0;1];
        zb = R.g_b*z0;
        zhr{i} = R.b_hr{i}*zb;
        zhp{i} = R.b_hr{i}*R.hr_hp{i}*zb;
        zk{i} = R.b_hr{i}*R.hr_hp{i}*R.hp_k{i}*zb;
        
        
        Jv.g1{i} = dx.g1{i}; 
        Jv.g2{i} = dx.g2{i}; 
        Jv.g3{i} = dx.g3{i}; 

        Jw.g1{i} = zhr{i};
        Jw.g2{i} = zhp{i};
        Jw.g3{i} = zk{i};

    end

%% end