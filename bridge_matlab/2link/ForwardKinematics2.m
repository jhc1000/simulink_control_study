function [p] = ForwardKinematics2(q,p)
l1 = 100;
l2 = 100;
l3 = 100;
lb = 100;

R.g_b = rot('z',q.b);
R.b_hp1 = rot('x',pi)*rot('z',pi/2)*rot('z',q.hp1);
R.b_hp3 = rot('x',pi)*rot('z',pi/2)*rot('z',q.hp3);
R.hp_k1 = rot('x',pi)*rot('z',q.k1);
R.hp_k3 = rot('x',pi)*rot('z',q.k3);
R.k_w1 = eye(3);
R.k_w3 = eye(3);

p.b_hp1 = [lb, 0, 0]';
p.hp_k1 = [l2, 0, 0]';
p.k_w1 = [l3, 0, 0]';
p.b_hp3 = [-lb, 0, 0]';
p.hp_k3 = [l2, 0, 0]';
p.k_w3 = [l3, 0, 0]';

T.g_b = TransformationMatrix(R.g_b,p.b);
T.b_hp1 = TransformationMatrix(R.b_hp1,p.b_hp1);
T.b_hp3 = TransformationMatrix(R.b_hp3,p.b_hp3);
T.hp_k1 = TransformationMatrix(R.hp_k1,p.hp_k1);
T.hp_k3 = TransformationMatrix(R.hp_k3,p.hp_k3);
T.k_w1 = TransformationMatrix(R.k_w1,p.k_w1);
T.k_w3 = TransformationMatrix(R.k_w3,p.k_w3);

T.g_hp1 = T.g_b*T.b_hp1;
T.g_hp3 = T.g_b*T.b_hp3;
T.g_k1 = T.g_b*T.b_hp1*T.hp_k1;
T.g_k3 = T.g_b*T.b_hp3*T.hp_k3;
T.g_w1 = T.g_b*T.b_hp1*T.hp_k1*T.k_w1;
T.g_w3 = T.g_b*T.b_hp3*T.hp_k3*T.k_w3;

p.hp1 = T.g_hp1([1,2,3],[4]);
p.k1 = T.g_k1([1,2,3],[4]);
p.w1 = T.g_w1([1,2,3],[4]);
p.hp3 = T.g_hp3([1,2,3],[4]);
p.k3 = T.g_k3([1,2,3],[4]);
p.w3 = T.g_w3([1,2,3],[4]);

end

