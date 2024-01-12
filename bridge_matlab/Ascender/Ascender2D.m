close all
clear all
clc

%% parameter
L_rope = [1000*sqrt(2), 1000*sqrt(2)];
R_sheeve = 80;
W_ancher = 2000;
d_ancher = [50, 50];
x_bl = 50;
y_bl = 200;
q.abs = [0, 0];           % ABS encoder
q.rope = [pi/4, pi/4];    % RS422 encoder
q.motor = [0, 0];         % ROT encoder
q.base = [0, 0, 0];       % row pitch yaw
alpha = 0;
beta = 0;
p.base = [1000, -1000, 150]; % robot odometry
p.desired = [1000, -1000, 150]; % desired position
l.desired = [0, 0];


%% FK

l_rope = [L_rope(1) + R_sheeve*q.motor(1), L_rope(2) - R_sheeve*q.motor(2)];
alpha = pi/2 - (q.rope(1) + q.base(3));
beta = pi/2 - (q.rope(2) - q.base(3));

p.base(1) = l_rope(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
p.base(2) = -l_rope(2)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));

%% IK
l.desired(1) = sqrt((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)-x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
l.desired(2) = sqrt((p.desired(1)-W_ancher+x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)+x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);

q.motor_d(1) = (l.desired(1) - L_rope(1))/R_sheeve;
q.motor_d(2) = (L_rope(2) - l.desired(2))/R_sheeve;

alpha_d = acos((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))/l.desired(1));
beta_d = acos((W_ancher-p.desired(1)-x_bl*cos(q.base(3))+y_bl*sin(q.base(3)))/l.desired(2));

l_rope = [L_rope(1) + R_sheeve*q.motor_d(1), L_rope(2) - R_sheeve*q.motor_d(2)];
alpha = alpha_d;
beta = beta_d;

p.base(1) = l_rope(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
p.base(2) = -l_rope(1)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));


%% Visualization
figure;
h = plot(0,0);pt = get(h,'Parent');
xlim(pt,'manual');xlim(pt,[0 2000]);ylim(pt,'manual');ylim(pt,[-2000 0]);axis;grid on;%view(0,90);
title('Ascender 2D Simulation')
point1.x = 0;point1.y = 0;
point2.x = W_ancher;point2.y = 0;
point3.x = l_rope(1)*cos(alpha);point3.y = -l_rope(1)*sin(alpha);
point4.x = W_ancher - l_rope(2)*cos(beta);point4.y = -l_rope(2)*sin(beta);
point5.x = p.base(1);point5.y = p.base(2);
L1 = line([point1.x, point2.x],[point1.y,point2.y],'Color','k','LineWidth',2);
L2 = line([point1.x, point3.x],[point1.y,point3.y],'Color','r','LineWidth',2);
L3 = line([point2.x, point4.x],[point2.y,point4.y],'Color','r','LineWidth',2);
L4 = line([point3.x, point4.x],[point3.y,point4.y],'Color','k','LineWidth',2);
L5 = line([point3.x, point5.x],[point3.y,point5.y],'Color','k','LineWidth',2);
L6 = line([point4.x, point5.x],[point4.y,point5.y],'Color','k','LineWidth',2); 
pause(1);

%% Controller

l.desired(1) = sqrt((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)-x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
l.desired(2) = sqrt((p.desired(1)-W_ancher+x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)+x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);

q.motor_d(1) = (l.desired(1) - L_rope(1))/R_sheeve;
q.motor_d(2) = (L_rope(2) - l.desired(2))/R_sheeve;

alpha_d = acos((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))/l.desired(1));
beta_d = acos((W_ancher-p.desired(1)-x_bl*cos(q.base(3))+y_bl*sin(q.base(3)))/l.desired(2));


%% FK
l_rope = [L_rope(1) + R_sheeve*q.motor_d(1), L_rope(2) - R_sheeve*q.motor_d(2)];
alpha = alpha_d;
beta = beta_d;

p.base(1) = l_rope(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
p.base(2) = -l_rope(1)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));

%% Visualization
hold on
point1.x = 0;point1.y = 0;
point2.x = W_ancher;point2.y = 0;
point3.x = l_rope(1)*cos(alpha);point3.y = -l_rope(1)*sin(alpha);
point4.x = W_ancher - l_rope(2)*cos(beta);point4.y = -l_rope(2)*sin(beta);
point5.x = p.base(1);point5.y = p.base(2);
set(L1,'YData',[point1.y,point2.y],'XData',[point1.x,point2.x]);
set(L2,'YData',[point1.y,point3.y],'XData',[point1.x,point3.x]);
set(L3,'YData',[point2.y,point4.y],'XData',[point2.x,point4.x]);
set(L4,'YData',[point3.y,point4.y],'XData',[point3.x,point4.x]);
set(L5,'YData',[point3.y,point5.y],'XData',[point3.x,point5.x]);
set(L6,'YData',[point4.y,point5.y],'XData',[point4.x,point5.x]);
plot(p.base(1),p.base(2),'ob');
pause(1);

%% simualation
hold on
p.desired = [1500, -1000, 0];
plot(p.desired(1),p.desired(2),'or');
x=1000:10:1500+10;
for i=1:1:numel(x)-1
    % Controller
    p.desired = [x(i), -1000, 0];
    l.desired(1) = sqrt((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)-x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
    l.desired(2) = sqrt((p.desired(1)-W_ancher+x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)+x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
    
    q.motor_d(1) = (l.desired(1) - L_rope(1))/R_sheeve;
    q.motor_d(2) = (L_rope(2) - l.desired(2))/R_sheeve;

    q.motor_acc1(1, i) = q.motor_d(1) * 180/pi;
    q.motor_acc1(2, i) = q.motor_d(2) * 180/pi;
    
    alpha_d = acos((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))/l.desired(1));
    beta_d = acos((W_ancher-p.desired(1)-x_bl*cos(q.base(3))+y_bl*sin(q.base(3)))/l.desired(2));
    
    % position estimation
    l_rope = [L_rope(1) + R_sheeve*q.motor_d(1), L_rope(2) - R_sheeve*q.motor_d(2)];
    alpha = alpha_d;
    beta = beta_d;
    
    p.base(1) = l_rope(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
    p.base(2) = -l_rope(1)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));
    
    % visualization
    hold on
    point1.x = 0;point1.y = 0;
    point2.x = W_ancher;point2.y = 0;
    point3.x = l_rope(1)*cos(alpha);point3.y = -l_rope(1)*sin(alpha);
    point4.x = W_ancher - l_rope(2)*cos(beta);point4.y = -l_rope(2)*sin(beta);
    point5.x = p.base(1);point5.y = p.base(2);
    set(L1,'YData',[point1.y,point2.y],'XData',[point1.x,point2.x]);
    set(L2,'YData',[point1.y,point3.y],'XData',[point1.x,point3.x]);
    set(L3,'YData',[point2.y,point4.y],'XData',[point2.x,point4.x]);
    set(L4,'YData',[point3.y,point4.y],'XData',[point3.x,point4.x]);
    set(L5,'YData',[point3.y,point5.y],'XData',[point3.x,point5.x]);
    set(L6,'YData',[point4.y,point5.y],'XData',[point4.x,point5.x]);
    plot(p.base(1),p.base(2),'ob');
%     plot(x(i+1),p.base(2),'.r');
    pause(0.1);
end
pause(1);

p.desired = [1000, -500, 0];
plot(p.desired(1),p.desired(2),'or');
y=-1500:10:-500+10;
for i=1:1:numel(y)-1
    p.desired = [1000, y(i), 0];
    l.desired(1) = sqrt((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)-x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
    l.desired(2) = sqrt((p.desired(1)-W_ancher+x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)+x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
    
    q.motor_d(1) = (l.desired(1) - L_rope(1))/R_sheeve;
    q.motor_d(2) = (L_rope(2) - l.desired(2))/R_sheeve;

    q.motor_acc2(1, i) = q.motor_d(1) * 180/pi;
    q.motor_acc2(2, i) = q.motor_d(2) * 180/pi;
    
    alpha_d = acos((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))/l.desired(1));
    beta_d = acos((W_ancher-p.desired(1)-x_bl*cos(q.base(3))+y_bl*sin(q.base(3)))/l.desired(2));

    l_rope = [L_rope(1) + R_sheeve*q.motor_d(1), L_rope(2) - R_sheeve*q.motor_d(2)];
    alpha = alpha_d;
    beta = beta_d;
    
    p.base(1) = l_rope(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
    p.base(2) = -l_rope(1)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));

    hold on
    point1.x = 0;point1.y = 0;
    point2.x = W_ancher;point2.y = 0;
    point3.x = l_rope(1)*cos(alpha);point3.y = -l_rope(1)*sin(alpha);
    point4.x = W_ancher - l_rope(2)*cos(beta);point4.y = -l_rope(2)*sin(beta);
    point5.x = p.base(1);point5.y = p.base(2);
    set(L1,'YData',[point1.y,point2.y],'XData',[point1.x,point2.x]);
    set(L2,'YData',[point1.y,point3.y],'XData',[point1.x,point3.x]);
    set(L3,'YData',[point2.y,point4.y],'XData',[point2.x,point4.x]);
    set(L4,'YData',[point3.y,point4.y],'XData',[point3.x,point4.x]);
    set(L5,'YData',[point3.y,point5.y],'XData',[point3.x,point5.x]);
    set(L6,'YData',[point4.y,point5.y],'XData',[point4.x,point5.x]);
    plot(p.base(1),p.base(2),'ob');
%     plot(p.base(1),y(i+1),'.r');
    pause(0.1);
end
pause(1);


