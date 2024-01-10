function out = Ac3(u)
n = 3;
m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
% pdot1 = u(1); pdot2 = u(2); pdot3 = u(3); taud1 = u(4); taud2 = u(5); taud3 = u(6);
Ac_1 = zeros(n); 
Ac_2 = -1*eye(n); 
Ac_3 = zeros(n);
a = [-0.001; -0.002; -0.003];
Ac_4 = diag(a);
% Ac_4 = zeros(n);
Ac = [Ac_1, Ac_2; Ac_3, Ac_4];
% Ac =[0     0     0    -1     0     0;
%      0     0     0     0    -1     0;
%      0     0     0     0     0    -1;
%      0     0     0     0     0     0;
%      0     0     0     0     0     0;
%      0     0     0     0     0     0];
u_h = [u(1);u(2);u(3);u(4);u(5);u(6)];
out = Ac*u_h;
end

