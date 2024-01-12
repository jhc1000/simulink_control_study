close all
clear all
clc

% Parameter
q.b = 0.2;
q.w1 = pi/2;
q.w3 = pi/2;
q.k1 = 0;
q.k3 = 0;
q.hp1 = 0;
q.hp3 = 0;
l1 = 100;
l2 = 100;
l3 = 120;
lb = 100;

p.b = [100, 150, 0]';
p.w1 = [2*lb, 0, 0]';
p.w3 = [0, 0, 0]';
p.k1 = [2*lb, l3, 0]';
p.k3 = [0, l3, 0]';
p.hp1 = [2*lb, 220, 0]';
p.hp3 = [0, 220, 0]';


% foward
% p.k1 = p.w1 + [-l3*cos(q.w1),l3*sin(q.w1),0]';
% p.hp1 = p.k1 + [-l2*sin(q.b-q.hp1),l2*cos(q.b-q.hp1),0]';
% p.b = p.hp1 + [-lb*cos(q.b),-lb*sin(q.b),0]';
% 
% p.k3 = p.w3 + [-l3*cos(q.w3),l3*sin(q.w3),0]';
% p.hp3 = p.k3 + [-l2*sin(q.b-q.hp3),l2*cos(q.b-q.hp3),0]';
% p.b = p.hp3 + [lb*cos(q.b),lb*sin(q.b),0]';

% foward
% R.g_b = rot('z',q.b);
% R.b_hp1 = rot('x',pi)*rot('z',pi/2)*rot('z',q.hp1);
% R.b_hp3 = rot('x',pi)*rot('z',pi/2)*rot('z',q.hp3);
% R.hp_k1 = rot('x',pi)*rot('z',q.k1);
% R.hp_k3 = rot('x',pi)*rot('z',q.k3);
% R.k_w1 = eye(3);
% R.k_w3 = eye(3);
% 
% p.b_hp1 = [lb, 0, 0]';
% p.hp_k1 = [l2, 0, 0]';
% p.k_w1 = [l3, 0, 0]';
% p.b_hp3 = [-lb, 0, 0]';
% p.hp_k3 = [l2, 0, 0]';
% p.k_w3 = [l3, 0, 0]';
% 
% T.g_b = TransformationMatrix(R.g_b,p.b);
% T.b_hp1 = TransformationMatrix(R.b_hp1,p.b_hp1);
% T.b_hp3 = TransformationMatrix(R.b_hp3,p.b_hp3);
% T.hp_k1 = TransformationMatrix(R.hp_k1,p.hp_k1);
% T.hp_k3 = TransformationMatrix(R.hp_k3,p.hp_k3);
% T.k_w1 = TransformationMatrix(R.k_w1,p.k_w1);
% T.k_w3 = TransformationMatrix(R.k_w3,p.k_w3);
% 
% T.g_hp1 = T.g_b*T.b_hp1;
% T.g_hp3 = T.g_b*T.b_hp3;
% T.g_k1 = T.g_b*T.b_hp1*T.hp_k1;
% T.g_k3 = T.g_b*T.b_hp3*T.hp_k3;
% T.g_w1 = T.g_b*T.b_hp1*T.hp_k1*T.k_w1;
% T.g_w3 = T.g_b*T.b_hp3*T.hp_k3*T.k_w3;
% 
% p.hp1 = T.g_hp1([1,2,3],[4]);
% p.k1 = T.g_k1([1,2,3],[4]);
% p.w1 = T.g_w1([1,2,3],[4]);
% p.hp3 = T.g_hp3([1,2,3],[4]);
% p.k3 = T.g_k3([1,2,3],[4]);
% p.w3 = T.g_w3([1,2,3],[4]);

% inverse 
gamma1 = atan2(p.w1(1)-((p.b(1)+lb*cos(q.b))),(p.b(2)+lb*sin(q.b)-p.w1(2)));
k1 = sqrt((p.b(1)+lb*cos(q.b)-p.w1(1))^2+(p.b(2)+lb*sin(q.b)-p.w1(2))^2);
q.k1 = acos((k1^2-l2^2-l3^2)/(2*l2*l3));
k2 = l3*sin(q.k1);
alpha1 = asin(k2/k1);
q.hp1 = alpha1+q.b-gamma1;
q.w1 = pi/2+q.hp1-q.b-q.k1;

gamma3 = atan2(p.w3(1)-((p.b(1)-lb*cos(q.b))),(p.b(2)-lb*sin(q.b)-p.w3(2)));
k3 = sqrt((p.b(1)-lb*cos(q.b)-p.w3(1))^2+(p.b(2)-lb*sin(q.b)-p.w3(2))^2);
q.k3 = acos((k3^2-l2^2-l3^2)/(2*l2*l3));
k4 = l3*sin(q.k3);
alpha3 = asin(k4/k3);
q.hp3 = alpha3+q.b-gamma3;
q.w3 = pi/2+q.hp3-q.b-q.k3;

% foward
p.k1 = p.w1 + [-l3*cos(q.w1),l3*sin(q.w1),0]';
p.hp1 = p.k1 + [-l2*sin(q.b-q.hp1),l2*cos(q.b-q.hp1),0]';
p.b = p.hp1 + [-lb*cos(q.b),-lb*sin(q.b),0]';

p.k3 = p.w3 + [-l3*cos(q.w3),l3*sin(q.w3),0]';
p.hp3 = p.k3 + [-l2*sin(q.b-q.hp3),l2*cos(q.b-q.hp3),0]';
p.b = p.hp3 + [lb*cos(q.b),lb*sin(q.b),0]';


point1.x = p.w1(1);point1.y= p.w1(2);point1.z = p.w1(3);
point2.x = p.k1(1);point2.y= p.k1(2);point2.z = p.k1(3);
point3.x = p.hp1(1);point3.y= p.hp1(2);point3.z = p.hp1(3);
point4.x = p.b(1);point4.y= p.b(2);point4.z = p.b(3);
point5.x = p.hp3(1);point5.y= p.hp3(2);point5.z = p.hp3(3);
point6.x = p.k3(1);point6.y= p.k3(2);point6.z = p.k3(3);
point7.x = p.w3(1);point7.y= p.w3(2);point7.z = p.w3(3);

figure;
xlim([-300,300]);ylim([-300,300]);grid on;
title("2D Robot Base Movement");
hold on
L1 = line([point1.x, point2.x],[point1.y,point2.y],[point1.z, point2.z],'Color','r','LineWidth',2);
L2 = line([point2.x, point3.x],[point2.y,point3.y],[point2.z, point3.z],'Color','g','LineWidth',2);
L3 = line([point3.x, point4.x],[point3.y,point4.y],[point3.z, point4.z],'Color','b','LineWidth',2);
L4 = line([point4.x, point5.x],[point4.y,point5.y],[point4.z, point5.z],'Color','b','LineWidth',2);
L5 = line([point5.x, point6.x],[point5.y,point6.y],[point5.z, point6.z],'Color','g','LineWidth',2);
L6 = line([point6.x, point7.x],[point6.y,point7.y],[point6.z, point7.z],'Color','r','LineWidth',2);
pause(1);

dt = 0.1;
t = 0:dt:10;

xc = 100;
yc = 150;
r = 40;

x = xc + r*cos(pi/5*t);
y = yc + r*sin(pi/5*t);

hold on
for k = 1:length(t)
  
    p.b = [x(k), y(k), 0]';
    q.b = 0.2*cos(pi/5*k);
    

    % inverse 
    gamma1 = atan2(p.w1(1)-((p.b(1)+lb*cos(q.b))),(p.b(2)+lb*sin(q.b)-p.w1(2)));
    k1 = sqrt((p.b(1)+lb*cos(q.b)-p.w1(1))^2+(p.b(2)+lb*sin(q.b)-p.w1(2))^2);
    q.k1 = acos((k1^2-l2^2-l3^2)/(2*l2*l3));
    k2 = l3*sin(q.k1);
    alpha1 = asin(k2/k1);
    q.hp1 = alpha1+q.b-gamma1;
    q.w1 = pi/2+q.hp1-q.b-q.k1;
    
    gamma3 = atan2(p.w3(1)-((p.b(1)-lb*cos(q.b))),(p.b(2)-lb*sin(q.b)-p.w3(2)));
    k3 = sqrt((p.b(1)-lb*cos(q.b)-p.w3(1))^2+(p.b(2)-lb*sin(q.b)-p.w3(2))^2);
    q.k3 = acos((k3^2-l2^2-l3^2)/(2*l2*l3));
    k4 = l3*sin(q.k3);
    alpha3 = asin(k4/k3);
    q.hp3 = alpha3+q.b-gamma3;
    q.w3 = pi/2+q.hp3-q.b-q.k3;
    
    % foward
    p.k1 = p.w1 + [-l3*cos(q.w1),l3*sin(q.w1),0]';
    p.hp1 = p.k1 + [-l2*sin(q.b-q.hp1),l2*cos(q.b-q.hp1),0]';
    p.b = p.hp1 + [-lb*cos(q.b),-lb*sin(q.b),0]';
    
    p.k3 = p.w3 + [-l3*cos(q.w3),l3*sin(q.w3),0]';
    p.hp3 = p.k3 + [-l2*sin(q.b-q.hp3),l2*cos(q.b-q.hp3),0]';
    p.b = p.hp3 + [lb*cos(q.b),lb*sin(q.b),0]';
    

    point1.x = p.w1(1);point1.y= p.w1(2);point1.z = p.w1(3);
    point2.x = p.k1(1);point2.y= p.k1(2);point2.z = p.k1(3);
    point3.x = p.hp1(1);point3.y= p.hp1(2);point3.z = p.hp1(3);
    point4.x = p.b(1);point4.y= p.b(2);point4.z = p.b(3);
    point5.x = p.hp3(1);point5.y= p.hp3(2);point5.z = p.hp3(3);
    point6.x = p.k3(1);point6.y= p.k3(2);point6.z = p.k3(3);
    point7.x = p.w3(1);point7.y= p.w3(2);point7.z = p.w3(3);
    set(L1,'ZData',[0,0],'YData',[point1.y,point2.y],'XData',[point1.x, point2.x]);
    set(L2,'ZData',[0,0],'YData',[point2.y,point3.y],'XData',[point2.x, point3.x]);
    set(L3,'ZData',[0,0],'YData',[point3.y,point4.y],'XData',[point3.x, point4.x]);
    set(L4,'ZData',[0,0],'YData',[point4.y,point5.y],'XData',[point4.x, point5.x]);
    set(L5,'ZData',[0,0],'YData',[point5.y,point6.y],'XData',[point5.x, point6.x]);
    set(L6,'ZData',[0,0],'YData',[point6.y,point7.y],'XData',[point6.x, point7.x]);
    plot(point4.x,point4.y,'k.');
    pause(dt);
end