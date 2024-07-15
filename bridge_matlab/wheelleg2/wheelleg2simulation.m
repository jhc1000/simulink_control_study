close all
clear all
clc

% Parameter
self.p = [0; 0; 0];
self.q = [0; 0; 0];
q.hr = [deg2rad(90); deg2rad(-90); deg2rad(-90); deg2rad(90)];
q.hr = [deg2rad(45); deg2rad(-45); deg2rad(-45); deg2rad(45)];
q.hp = [deg2rad(0); deg2rad(0); deg2rad(0); deg2rad(0)];
q.k = [deg2rad(0); deg2rad(0); deg2rad(0); deg2rad(0)];
q.w = [deg2rad(0); deg2rad(0); deg2rad(0); deg2rad(0)];
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

% Forward Kinematics
p = ForwardKinematics2(q,self);

% Inverse Kinematics
q = InverseKinematics2(p,self);

% ForwardKinematics Simulation
p = ForwardKinematics2(q,self);
figure;
h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
xlim(pt,[-1000 1000]);ylim(pt,'manual');ylim(pt,[-1000 1000]);zlim(pt,'manual');
zlim(pt,[-1000 1000]);axis;grid on;%view(0,90);
title('Wheeleg ForwardKinematics Simulation')
for i = 1:4
point1.x{i} = p.b_hr{i}(1);point1.y{i} = p.b_hr{i}(2);point1.z{i} = p.b_hr{i}(3);
point2.x{i} = p.b_hp{i}(1);point2.y{i} = p.b_hp{i}(2);point2.z{i} = p.b_hp{i}(3);
point3.x{i} = p.b_k{i}(1);point3.y{i} = p.b_k{i}(2);point3.z{i} = p.b_k{i}(3);
point4.x{i} = p.b_w{i}(1);point4.y{i} = p.b_w{i}(2);point4.z{i} = p.b_w{i}(3);
L1{i} = line([point1.x{i}, point2.x{i}],[point1.y{i},point2.y{i}],[point1.z{i},point2.z{i}],'Color','r','LineWidth',2);
L2{i} = line([point2.x{i}, point3.x{i}],[point2.y{i},point3.y{i}],[point2.z{i},point3.z{i}],'Color','b','LineWidth',2);
L3{i} = line([point3.x{i}, point4.x{i}],[point3.y{i},point4.y{i}],[point3.z{i},point4.z{i}],'Color','g','LineWidth',2);
end
Lb{1} = line([point1.x{1}, point1.x{2}],[point1.y{1},point1.y{2}],[point1.z{1},point1.z{2}],'Color','k','LineWidth',1);
Lb{2} = line([point1.x{2}, point1.x{4}],[point1.y{2},point1.y{4}],[point1.z{2},point1.z{4}],'Color','k','LineWidth',1);
Lb{3} = line([point1.x{3}, point1.x{4}],[point1.y{3},point1.y{4}],[point1.z{3},point1.z{4}],'Color','k','LineWidth',1);
Lb{4} = line([point1.x{3}, point1.x{1}],[point1.y{3},point1.y{1}],[point1.z{3},point1.z{1}],'Color','k','LineWidth',1);
pause(1);

% ForwardKinematics Simulation
% for theta3 = 0:0.1:(pi/2)
%     q.k = [theta3;theta3;theta3;theta3];
%     p = ForwardKinematics(q,self);
%     q = InverseKinematics(p,self);
%     p = ForwardKinematics(q,self);
%     hold on;
%     for i =1:4
%     point1.x{i} = p.b_hr{i}(1);point1.y{i} = p.b_hr{i}(2);point1.z{i} = p.b_hr{i}(3);
%     point2.x{i} = p.b_hp{i}(1);point2.y{i} = p.b_hp{i}(2);point2.z{i} = p.b_hp{i}(3);
%     point3.x{i} = p.b_k{i}(1);point3.y{i} = p.b_k{i}(2);point3.z{i} = p.b_k{i}(3);
%     point4.x{i} = p.b_w{i}(1);point4.y{i} = p.b_w{i}(2);point4.z{i} = p.b_w{i}(3);
%     set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
%     set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
%     set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
%     plot3(point4.x{i},point4.y{i},point4.z{i},'ok');
%     end
% %     pause(0.1);
% end
% pause(1);

% for theta1 = 0:0.1:(pi/4)
%     q.hr = [n2(1)*theta1;n2(2)*theta1;n2(3)*theta1;n2(4)*theta1];
%     p = ForwardKinematics(q,self);
%     q = InverseKinematics(p,self);
%     p = ForwardKinematics(q,self);
%     hold on;
%     for i =1:4
%     point1.x{i} = p.b_hr{i}(1);point1.y{i} = p.b_hr{i}(2);point1.z{i} = p.b_hr{i}(3);
%     point2.x{i} = p.b_hp{i}(1);point2.y{i} = p.b_hp{i}(2);point2.z{i} = p.b_hp{i}(3);
%     point3.x{i} = p.b_k{i}(1);point3.y{i} = p.b_k{i}(2);point3.z{i} = p.b_k{i}(3);
%     point4.x{i} = p.b_w{i}(1);point4.y{i} = p.b_w{i}(2);point4.z{i} = p.b_w{i}(3);
%     set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
%     set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);     
%     set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
%     plot3(point4.x{i},point4.y{i},point4.z{i},'ok')
%     end
%     pause(0.1);
% end 
% pause(1);

% for theta2 = 0:0.1:(pi/4)
%     q.hp = [theta2;theta2;theta2;theta2];
%     p = ForwardKinematics(q,self);
%     q = InverseKinematics(p,self);
%     p = ForwardKinematics(q,self);
%     hold on;
%     for i =1:4
%     point1.x{i} = p.b_hr{i}(1);point1.y{i} = p.b_hr{i}(2);point1.z{i} = p.b_hr{i}(3);     
%     point2.x{i} = p.b_hp{i}(1);point2.y{i} = p.b_hp{i}(2);point2.z{i} = p.b_hp{i}(3);
%     point3.x{i} = p.b_k{i}(1);point3.y{i} = p.b_k{i}(2);point3.z{i} = p.b_k{i}(3);
%     point4.x{i} = p.b_w{i}(1);point4.y{i} = p.b_w{i}(2);point4.z{i} = p.b_w{i}(3);
%     set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
%     set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
%     set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
%     plot3(point4.x{i},point4.y{i},point4.z{i},'ok');
%     end
% %     pause(0.1);
% end 
% pause(1);
% 
% % hold off
% 
% % InverseKinematics Simulation
% pin.b_w=p.b_w;
% figure;
% h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
% xlim(pt,[-1000 1000]);ylim(pt,'manual');ylim(pt,[-1000 1000]);zlim(pt,'manual');
% zlim(pt,[-1000 1000]);axis;grid on;%view(0,90);
% title('Wheeleg InverseKinematics Simulation')
% qin = InverseKinematics(pin,self);
% pin2 = ForwardKinematics(qin,self); 
% for i = 1:4
% point1.x{i} = pin2.b_hr{i}(1);point1.y{i} = pin2.b_hr{i}(2);point1.z{i} = pin2.b_hr{i}(3);
% point2.x{i} = pin2.b_hp{i}(1);point2.y{i} = pin2.b_hp{i}(2);point2.z{i} = pin2.b_hp{i}(3);
% point3.x{i} = pin2.b_k{i}(1);point3.y{i} = pin2.b_k{i}(2);point3.z{i} = pin2.b_k{i}(3);
% point4.x{i} = pin2.b_w{i}(1);point4.y{i} = pin2.b_w{i}(2);point4.z{i} = pin2.b_w{i}(3);
% L1{i} = line([point1.x{i}, point2.x{i}],[point1.y{i},point2.y{i}],[point1.z{i},point2.z{i}],'Color','r','LineWidth',2);
% L2{i} = line([point2.x{i}, point3.x{i}],[point2.y{i},point3.y{i}],[point2.z{i},point3.z{i}],'Color','b','LineWidth',2);
% L3{i} = line([point3.x{i}, point4.x{i}],[point3.y{i},point4.y{i}],[point3.z{i},point4.z{i}],'Color','g','LineWidth',2);
% end
% Lb{1} = line([point1.x{1}, point1.x{2}],[point1.y{1},point1.y{2}],[point1.z{1},point1.z{2}],'Color','k','LineWidth',1);
% Lb{2} = line([point1.x{2}, point1.x{4}],[point1.y{2},point1.y{4}],[point1.z{2},point1.z{4}],'Color','k','LineWidth',1);
% Lb{3} = line([point1.x{3}, point1.x{4}],[point1.y{3},point1.y{4}],[point1.z{3},point1.z{4}],'Color','k','LineWidth',1);
% Lb{4} = line([point1.x{3}, point1.x{1}],[point1.y{3},point1.y{1}],[point1.z{3},point1.z{1}],'Color','k','LineWidth',1);
% pause(1);
% 
% dt = 0.1;
% t = 0:dt:10;
% 
% xc = lb;
% yc = 223.6;
% zc = -500;
% r = 100;
% 
% x = r*cos(pi/5*t);
% y = yc;
% z = zc + r*sin(pi/5*t);
% 
% hold on
% for k = 1:length(t)
%     for i = 1:4
%         pin.b_w{i} = [n1(i)*xc+x(k) n2(i)*y z(k)];
%     end
% 
%     qin = InverseKinematics(pin,self);
%     pin2 = ForwardKinematics(qin,self); 
%     for i = 1:4
%     point1.x{i} = pin2.b_hr{i}(1);point1.y{i} = pin2.b_hr{i}(2);point1.z{i} = pin2.b_hr{i}(3);
%     point2.x{i} = pin2.b_hp{i}(1);point2.y{i} = pin2.b_hp{i}(2);point2.z{i} = pin2.b_hp{i}(3);
%     point3.x{i} = pin2.b_k{i}(1);point3.y{i} = pin2.b_k{i}(2);point3.z{i} = pin2.b_k{i}(3);
%     point4.x{i} = pin2.b_w{i}(1);point4.y{i} = pin2.b_w{i}(2);point4.z{i} = pin2.b_w{i}(3);
%     set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
%     set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
%     set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
%     plot3(point4.x{i},point4.y{i},point4.z{i},'.k');
%     end
%     pause(dt);
% end
% 
% % Base frame movement
% self.p = [300, 0, -200]';
% self.q = [0, 0.3, 0]';
% figure;
% h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
% xlim(pt,[-1000 1000]);ylim(pt,'manual');ylim(pt,[-1000 1000]);zlim(pt,'manual');
% zlim(pt,[-1000 1000]);axis;grid on;%view(0,90);
% title('Wheeleg Base frame movement Simulation')
% pin2 = ForwardKinematics2(qin,self); 
% for i = 1:4
% point1.x{i} = pin2.g_hr{i}(1);point1.y{i} = pin2.g_hr{i}(2);point1.z{i} = pin2.g_hr{i}(3);
% point2.x{i} = pin2.g_hp{i}(1);point2.y{i} = pin2.g_hp{i}(2);point2.z{i} = pin2.g_hp{i}(3);
% point3.x{i} = pin2.g_k{i}(1);point3.y{i} = pin2.g_k{i}(2);point3.z{i} = pin2.g_k{i}(3);
% point4.x{i} = pin2.g_w{i}(1);point4.y{i} = pin2.g_w{i}(2);point4.z{i} = pin2.g_w{i}(3);
% L1{i} = line([point1.x{i}, point2.x{i}],[point1.y{i},point2.y{i}],[point1.z{i},point2.z{i}],'Color','r','LineWidth',2);
% L2{i} = line([point2.x{i}, point3.x{i}],[point2.y{i},point3.y{i}],[point2.z{i},point3.z{i}],'Color','b','LineWidth',2);
% L3{i} = line([point3.x{i}, point4.x{i}],[point3.y{i},point4.y{i}],[point3.z{i},point4.z{i}],'Color','g','LineWidth',2);
% end
% Lb{1} = line([point1.x{1}, point1.x{2}],[point1.y{1},point1.y{2}],[point1.z{1},point1.z{2}],'Color','k','LineWidth',1);
% Lb{2} = line([point1.x{2}, point1.x{4}],[point1.y{2},point1.y{4}],[point1.z{2},point1.z{4}],'Color','k','LineWidth',1);
% Lb{3} = line([point1.x{3}, point1.x{4}],[point1.y{3},point1.y{4}],[point1.z{3},point1.z{4}],'Color','k','LineWidth',1);
% Lb{4} = line([point1.x{3}, point1.x{1}],[point1.y{3},point1.y{1}],[point1.z{3},point1.z{1}],'Color','k','LineWidth',1);
% pause(1);
% 
% dt = 0.1;
% t = 0:dt:10;
% 
% xc = 0;
% yc = 0;
% zc = 0;
% 
% x = 50*t;
% y = yc;
% z = zc;
% 
% hold on
% for k = 1:length(t)
% 
%     self.p = [x(k), y, z]';
%     self.q = [0, 0.2*cos(pi/5*k), 0]';
% 
%     pin2 = ForwardKinematics2(qin,self); 
%     for i = 1:4
%     point1.x{i} = pin2.g_hr{i}(1);point1.y{i} = pin2.g_hr{i}(2);point1.z{i} = pin2.g_hr{i}(3);
%     point2.x{i} = pin2.g_hp{i}(1);point2.y{i} = pin2.g_hp{i}(2);point2.z{i} = pin2.g_hp{i}(3);
%     point3.x{i} = pin2.g_k{i}(1);point3.y{i} = pin2.g_k{i}(2);point3.z{i} = pin2.g_k{i}(3);
%     point4.x{i} = pin2.g_w{i}(1);point4.y{i} = pin2.g_w{i}(2);point4.z{i} = pin2.g_w{i}(3);
%     set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
%     set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
%     set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
%     plot3(point4.x{i},point4.y{i},point4.z{i},'.k');
%     end
%     set(Lb{1},'ZData',[point1.z{1},point1.z{2}],'YData',[point1.y{1},point1.y{2}],'XData',[point1.x{1}, point1.x{2}]);
%     set(Lb{2},'ZData',[point1.z{2},point1.z{4}],'YData',[point1.y{2},point1.y{4}],'XData',[point1.x{2}, point1.x{4}]);
%     set(Lb{3},'ZData',[point1.z{3},point1.z{4}],'YData',[point1.y{3},point1.y{4}],'XData',[point1.x{3}, point1.x{4}]);
%     set(Lb{4},'ZData',[point1.z{3},point1.z{1}],'YData',[point1.y{3},point1.y{1}],'XData',[point1.x{3}, point1.x{1}]);
%     pause(dt);
% end
% 

% base movement

self.p = [0; 0; -100];
self.q = [0; 0; 0];
q.hr = [0; 0; 0; 0];
q.hp = [pi/5; pi/5; pi/5; pi/5];
q.k = [pi/2; pi/2; pi/2; pi/2];
q.w = [0; 0; 0; 0];

p = ForwardKinematics2(q,self);

figure;
h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
xlim(pt,[-1000 1000]);ylim(pt,'manual');ylim(pt,[-1000 1000]);zlim(pt,'manual');
zlim(pt,[-1000 1000]);axis;grid on;%view(0,90);
title('Wheeleg InverseKinematics Simulation')
for i = 1:4
point1.x{i} = p.g_hr{i}(1);point1.y{i} = p.g_hr{i}(2);point1.z{i} = p.g_hr{i}(3);
point2.x{i} = p.g_hp{i}(1);point2.y{i} = p.g_hp{i}(2);point2.z{i} = p.g_hp{i}(3);
point3.x{i} = p.g_k{i}(1);point3.y{i} = p.g_k{i}(2);point3.z{i} = p.g_k{i}(3);
point4.x{i} = p.g_w{i}(1);point4.y{i} = p.g_w{i}(2);point4.z{i} = p.g_w{i}(3);
L1{i} = line([point1.x{i}, point2.x{i}],[point1.y{i},point2.y{i}],[point1.z{i},point2.z{i}],'Color','r','LineWidth',2);
L2{i} = line([point2.x{i}, point3.x{i}],[point2.y{i},point3.y{i}],[point2.z{i},point3.z{i}],'Color','b','LineWidth',2);
L3{i} = line([point3.x{i}, point4.x{i}],[point3.y{i},point4.y{i}],[point3.z{i},point4.z{i}],'Color','g','LineWidth',2);
end
Lb{1} = line([point1.x{1}, point1.x{2}],[point1.y{1},point1.y{2}],[point1.z{1},point1.z{2}],'Color','k','LineWidth',1);
Lb{2} = line([point1.x{2}, point1.x{4}],[point1.y{2},point1.y{4}],[point1.z{2},point1.z{4}],'Color','k','LineWidth',1);
Lb{3} = line([point1.x{3}, point1.x{4}],[point1.y{3},point1.y{4}],[point1.z{3},point1.z{4}],'Color','k','LineWidth',1);
Lb{4} = line([point1.x{3}, point1.x{1}],[point1.y{3},point1.y{1}],[point1.z{3},point1.z{1}],'Color','k','LineWidth',1);
pause(1);

self.p = [0; 0; -100];
self.q = [0; 0; 0];

Rg_b = rotz(self.q(3))*roty(self.q(2))*rotx(self.q(1));
for i = 1:4
    p.b_w{i} = inv(Rg_b)*(p.g_w{i} - self.p);
end

q = InverseKinematics(p,self);
p = ForwardKinematics2(q,self);

figure;
hold on
h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
xlim(pt,[-700 700]);ylim(pt,'manual');ylim(pt,[-700 700]);zlim(pt,'manual');
zlim(pt,[-700 700]);axis;grid on;view(45,15);%view(80,45);
title('Wheeleg base movement Simulation')
for i = 1:4
point1.x{i} = p.g_hr{i}(1);point1.y{i} = p.g_hr{i}(2);point1.z{i} = p.g_hr{i}(3);
point2.x{i} = p.g_hp{i}(1);point2.y{i} = p.g_hp{i}(2);point2.z{i} = p.g_hp{i}(3);
point3.x{i} = p.g_k{i}(1);point3.y{i} = p.g_k{i}(2);point3.z{i} = p.g_k{i}(3);
point4.x{i} = p.g_w{i}(1);point4.y{i} = p.g_w{i}(2);point4.z{i} = p.g_w{i}(3);
L1{i} = line([point1.x{i}, point2.x{i}],[point1.y{i},point2.y{i}],[point1.z{i},point2.z{i}],'Color','r','LineWidth',2);
L2{i} = line([point2.x{i}, point3.x{i}],[point2.y{i},point3.y{i}],[point2.z{i},point3.z{i}],'Color','b','LineWidth',2);
L3{i} = line([point3.x{i}, point4.x{i}],[point3.y{i},point4.y{i}],[point3.z{i},point4.z{i}],'Color','g','LineWidth',2);
end
Lb{1} = line([point1.x{1}, point1.x{2}],[point1.y{1},point1.y{2}],[point1.z{1},point1.z{2}],'Color','k','LineWidth',1);
Lb{2} = line([point1.x{2}, point1.x{4}],[point1.y{2},point1.y{4}],[point1.z{2},point1.z{4}],'Color','k','LineWidth',1);
Lb{3} = line([point1.x{3}, point1.x{4}],[point1.y{3},point1.y{4}],[point1.z{3},point1.z{4}],'Color','k','LineWidth',1);
Lb{4} = line([point1.x{3}, point1.x{1}],[point1.y{3},point1.y{1}],[point1.z{3},point1.z{1}],'Color','k','LineWidth',1);
pause(1);

dt = 0.1;
t = 0:dt:10;

self.p = [0; 0; -100];
self.q = [0; 0; 0];

for i = 1:4
    a{i} = p.g_w{i};
end

hold on
for k = 1:length(t)
    self.p = [self.p(1)+0.3*k, 0, -100+50*sin(pi/5*k)]';
    for i = 1:4
        p.g_w{i} = p.g_w{i} + [0.3*k, 0, 0]';
    end
    self.q = [0, -0.2*cos(pi/5*k), 0]';

    self.p = [0, 0, -100+15*sin(pi/4*k)]';
    p.g_w{1} = a{1} + [0, 0, n1(1)*n2(1)*30*cos(pi/4*k)]';
    p.g_w{2} = a{2} + [0, 0, n1(2)*n2(2)*30*sin(pi/4*k)]';
    p.g_w{3} = a{3} + [0, 0, n1(3)*n2(3)*30*cos(pi/4*k)]';
    p.g_w{4} = a{4} + [0, 0, n1(4)*n2(4)*30*sin(pi/4*k)]';


    self.p = [0, 0, -100+50*sin(pi/5*k)]';
    self.q = [0, -0.2*cos(pi/5*k), 0]';
    self.q = [0, 0, 0.05*sin(pi/5*k)]';
    self.q = [0.01*sin(pi/5*k), 0, 0]';

    % base movement
    Rg_b = rotz(self.q(3))*roty(self.q(2))*rotx(self.q(1));
    for i = 1:4
        p.b_w{i} = inv(Rg_b)*(p.g_w{i} - self.p);
    end

    q = InverseKinematics2(p,self);
    p = ForwardKinematics2(q,self);

    for i = 1:4
    point1.x{i} = p.g_hr{i}(1);point1.y{i} = p.g_hr{i}(2);point1.z{i} = p.g_hr{i}(3);
    point2.x{i} = p.g_hp{i}(1);point2.y{i} = p.g_hp{i}(2);point2.z{i} = p.g_hp{i}(3);
    point3.x{i} = p.g_k{i}(1);point3.y{i} = p.g_k{i}(2);point3.z{i} = p.g_k{i}(3);
    point4.x{i} = p.g_w{i}(1);point4.y{i} = p.g_w{i}(2);point4.z{i} = p.g_w{i}(3);
    set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
    set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
    set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
    plot3(point4.x{i},point4.y{i},point4.z{i},'or');
%     plot3(point1.x{i},point1.y{i},point1.z{i},'or');
    end
    set(Lb{1},'ZData',[point1.z{1},point1.z{2}],'YData',[point1.y{1},point1.y{2}],'XData',[point1.x{1}, point1.x{2}]);
    set(Lb{2},'ZData',[point1.z{2},point1.z{4}],'YData',[point1.y{2},point1.y{4}],'XData',[point1.x{2}, point1.x{4}]);
    set(Lb{3},'ZData',[point1.z{3},point1.z{4}],'YData',[point1.y{3},point1.y{4}],'XData',[point1.x{3}, point1.x{4}]);
    set(Lb{4},'ZData',[point1.z{3},point1.z{1}],'YData',[point1.y{3},point1.y{1}],'XData',[point1.x{3}, point1.x{1}]);
    pause(dt);
end


