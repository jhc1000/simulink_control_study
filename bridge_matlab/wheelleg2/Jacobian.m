function [Jv,Jw] = Jacobian(eff,p,q,self)
    
    %% Base Coordinate     
    R.g_b = rotz(self.q(3))*roty(self.q(2))*rotx(self.q(1));

    %% define Rotation matrix
    for i = 1:4
        R.b_hr{i} = [0 -1 0;1 0 0;0 0 1]*rotz(q.hr(i));
        R.hr_hp{i} = [0 0 1;0 1 0;-1 0 0]*rotz(q.hp(i));
        R.hp_k{i} = [1 0 0;0 -1 0;0 0 -1]*rotz(q.k(i));
        R.k_w{i} = [-1 0 0;0 1 0;0 0 -1];

        %% Define end effector Jacobian       
        z0 = [0;0;1];
        zb = R.g_b*z0;
        zhr{i} = R.b_hr{i}*zb;
        zhp{i} = R.b_hr{i}*R.hr_hp{i}*zb;
        zk{i} = R.b_hr{i}*R.hr_hp{i}*R.hp_k{i}*zb;
    
        Jv.hr{i} = cros(zb,(eff{i}-self.p));
        Jv.hp{i} = cros(zhr,(eff{i}-p.g_hr));
        Jv.k{i} = cros(zhp,(eff{i}-p.g_hp));
        Jv.w{i} = cros(zk,(eff{i}-p.g_k));
    
        Jw.hr{i} = zb;
        Jw.hp{i} = zhr{i};
        Jw.k{i} = zhp{i};
        Jw.w{i} = zk{i};
    end
    %% define jacobian for dynamics     
    for i = 1:4
        


    end
end