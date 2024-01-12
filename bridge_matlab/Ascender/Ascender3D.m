close all
clear all
clc

%% parameter
L_rope = [2000*sqrt(2), 2000*sqrt(2)];
R_sheeve = 80;
W_ancher = 2000;
d_ancher = [10, 10];
x_bl = 50;
y_bl = 200;
q.abs = [0.0860, 0.0860];           % ABS encoder
q.rope = [pi/4, pi/4];    % RS422 encoder
q.motor = [0, 0];         % ROT encoder
q.base = [0, 0, 0];       % row pitch yaw
alpha = 0;
beta = 0;
p.base = [1000, -1000, 100]; % robot odometry
p.desired = [1000, -1000, 100]; % desired position
l.desired = [0, 0];

%% IK

q.abs_d(1) = acot((-p.desired(2) + x_bl*sin(q.base(3) - y_bl*cos(q.base(3))))/(p.base(3)-d_ancher(1)));
q.abs_d(2) = acot((-p.desired(2) + x_bl*sin(q.base(3) - y_bl*cos(q.base(3))))/(p.base(3)-d_ancher(2)));

l.ddesired(1) = sqrt((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)-x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
l.ddesired(2) = sqrt((p.desired(1)-W_ancher+x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)+x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);

alpha_d = acos((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))/l.ddesired(1));
beta_d = acos((W_ancher-p.desired(1)-x_bl*cos(q.base(3))+y_bl*sin(q.base(3)))/l.ddesired(2));

l.desired(1) = sqrt((p.base(3)-d_ancher(1))^2 + l.ddesired(1)^2);
l.desired(2) = sqrt((p.base(3)-d_ancher(2))^2 + l.ddesired(2)^2);

q.motor_d(1) = (l.desired(1) - L_rope(1))/R_sheeve;
q.motor_d(2) = (L_rope(2) - l.desired(2))/R_sheeve;

%% FK

l_rope = [L_rope(1) + R_sheeve*q.motor_d(1), L_rope(2) - R_sheeve*q.motor_d(2)];
alpha = alpha_d;
beta = beta_d;
q.abs(1) = q.abs_d(1);
q.abs(2) = q.abs_d(2);

h1 = (p.base(3)-d_ancher(1))*cot(q.abs(1));
h2 = (p.base(3)-d_ancher(2))*cot(q.abs(2));

% p.base(1) = h1*cot(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
% p.base(2) = -h1 + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));
p.base(3) = p.base(3);

p.base(1) = l.ddesired(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
p.base(2) = -l.ddesired(1)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));


%% Visualization
figure;
h = plot3(0,0,0);pt = get(h,'Parent');
xlim(pt,'manual');xlim(pt,[0 2000]);ylim(pt,'manual');ylim(pt,[-2000 0]);zlim(pt,'manual');
zlim(pt,[0 300]);axis;grid on;view(45,45);
title('Ascender 3D Simulation')
point1.x = 0;point1.y = 0;point1.z = d_ancher(1);
point2.x = W_ancher;point2.y = 0;point2.z = d_ancher(2);
point3.x = l.ddesired(1)*cos(alpha);point3.y = -l.ddesired(1)*sin(alpha);point3.z = p.base(3);
point4.x = W_ancher - l.ddesired(2)*cos(beta);point4.y = -l.ddesired(2)*sin(beta);point4.z = p.base(3);
point5.x = p.base(1);point5.y = p.base(2);point5.z = p.base(3);
L0 = line([0,W_ancher],[0,0],[0,0],'Color','k','LineWidth',2);
L1 = line([point1.x, point2.x],[point1.y,point2.y],[point1.z,point2.z],'Color','k','LineWidth',2);
L2 = line([point1.x, point3.x],[point1.y,point3.y],[point1.z,point3.z],'Color','r','LineWidth',2);
L3 = line([point2.x, point4.x],[point2.y,point4.y],[point2.z,point4.z],'Color','r','LineWidth',2);  
L4 = line([point3.x, point4.x],[point3.y,point4.y],[point3.z,point4.z],'Color','k','LineWidth',2);
L5 = line([point3.x, point5.x],[point3.y,point5.y],[point3.z,point5.z],'Color','k','LineWidth',2);
L6 = line([point4.x, point5.x],[point4.y,point5.y],[point4.z,point5.z],'Color','k','LineWidth',2); 
pause(1);

%% Controller

q.abs_d(1) = acot((-p.desired(2) + x_bl*sin(q.base(3) - y_bl*cos(q.base(3))))/(p.base(3)-d_ancher(1)));
q.abs_d(2) = acot((-p.desired(2) + x_bl*sin(q.base(3) - y_bl*cos(q.base(3))))/(p.base(3)-d_ancher(2)));

l.ddesired(1) = sqrt((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)-x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
l.ddesired(2) = sqrt((p.desired(1)-W_ancher+x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)+x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);

alpha_d = acos((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))/l.ddesired(1));
beta_d = acos((W_ancher-p.desired(1)-x_bl*cos(q.base(3))+y_bl*sin(q.base(3)))/l.ddesired(2));

l.desired(1) = sqrt((p.base(3)-d_ancher(1))^2 + l.ddesired(1)^2);
l.desired(2) = sqrt((p.base(3)-d_ancher(2))^2 + l.ddesired(2)^2);

q.motor_d(1) = (l.desired(1) - L_rope(1))/R_sheeve;
q.motor_d(2) = (L_rope(2) - l.desired(2))/R_sheeve;

%% FK
l_rope = [L_rope(1) + R_sheeve*q.motor_d(1), L_rope(2) - R_sheeve*q.motor_d(2)];
alpha = alpha_d;
beta = beta_d;
q.abs(1) = q.abs_d(1);
q.abs(2) = q.abs_d(2);

h1 = (p.base(3)-d_ancher(1))*cot(q.abs(1));
h2 = (p.base(3)-d_ancher(2))*cot(q.abs(2));

% p.base(1) = h1*cot(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
% p.base(2) = -h1 + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));
p.base(3) = p.base(3);

p.base(1) = l.ddesired(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
p.base(2) = -l.ddesired(1)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));

%% Visualization
hold on
point1.x = 0;point1.y = 0;point1.z = d_ancher(1);
point2.x = W_ancher;point2.y = 0;point2.z = d_ancher(2);
point3.x = l.ddesired(1)*cos(alpha);point3.y = -l.ddesired(1)*sin(alpha);point3.z = p.base(3);
point4.x = W_ancher - l.ddesired(2)*cos(beta);point4.y = -l.ddesired(2)*sin(beta);point4.z = p.base(3);
point5.x = p.base(1);point5.y = p.base(2);point5.z = p.base(3);
set(L1,'YData',[point1.y,point2.y],'XData',[point1.x,point2.x],'ZData',[point1.z,point2.z]);
set(L2,'YData',[point1.y,point3.y],'XData',[point1.x,point3.x],'ZData',[point1.z,point3.z]);
set(L3,'YData',[point2.y,point4.y],'XData',[point2.x,point4.x],'ZData',[point2.z,point4.z]);
set(L4,'YData',[point3.y,point4.y],'XData',[point3.x,point4.x],'ZData',[point3.z,point4.z]);
set(L5,'YData',[point3.y,point5.y],'XData',[point3.x,point5.x],'ZData',[point3.z,point5.z]);
set(L6,'YData',[point4.y,point5.y],'XData',[point4.x,point5.x],'ZData',[point4.z,point5.z]);
plot3(p.base(1),p.base(2),p.base(3),'ob');
pause(1);

%% simualation
hold on
p.desired = [1500, -1000, 100];
plot3(p.desired(1),p.desired(2),p.desired(3),'or');
x=1000:10:1500+10;
for i=1:1:numel(x)-1
    p.desired = [x(i), -1000, 100];

q.abs_d(1) = acot((-p.desired(2) + x_bl*sin(q.base(3) - y_bl*cos(q.base(3))))/(p.base(3)-d_ancher(1)));
q.abs_d(2) = acot((-p.desired(2) + x_bl*sin(q.base(3) - y_bl*cos(q.base(3))))/(p.base(3)-d_ancher(2)));

l.ddesired(1) = sqrt((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)-x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
l.ddesired(2) = sqrt((p.desired(1)-W_ancher+x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)+x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);

alpha_d = acos((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))/l.ddesired(1));
beta_d = acos((W_ancher-p.desired(1)-x_bl*cos(q.base(3))+y_bl*sin(q.base(3)))/l.ddesired(2));

l.desired(1) = sqrt((p.base(3)-d_ancher(1))^2 + l.ddesired(1)^2);
l.desired(2) = sqrt((p.base(3)-d_ancher(2))^2 + l.ddesired(2)^2);

q.motor_d(1) = (l.desired(1) - L_rope(1))/R_sheeve;
q.motor_d(2) = (L_rope(2) - l.desired(2))/R_sheeve;

    
    % position estimation
l_rope = [L_rope(1) + R_sheeve*q.motor_d(1), L_rope(2) - R_sheeve*q.motor_d(2)];
alpha = alpha_d;
beta = beta_d;
q.abs(1) = q.abs_d(1);
q.abs(2) = q.abs_d(2);

h1 = (p.base(3)-d_ancher(1))*cot(q.abs(1));
h2 = (p.base(3)-d_ancher(2))*cot(q.abs(2));

% p.base(1) = h1*cot(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
% p.base(2) = -h1 + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));
p.base(3) = p.base(3);

p.base(1) = l.ddesired(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
p.base(2) = -l.ddesired(1)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));
        
    % visualization
hold on
point1.x = 0;point1.y = 0;point1.z = d_ancher(1);
point2.x = W_ancher;point2.y = 0;point2.z = d_ancher(2);
point3.x = l.ddesired(1)*cos(alpha);point3.y = -l.ddesired(1)*sin(alpha);point3.z = p.base(3);
point4.x = W_ancher - l.ddesired(2)*cos(beta);point4.y = -l.ddesired(2)*sin(beta);point4.z = p.base(3);
point5.x = p.base(1);point5.y = p.base(2);point5.z = p.base(3);
set(L1,'YData',[point1.y,point2.y],'XData',[point1.x,point2.x],'ZData',[point1.z,point2.z]);
set(L2,'YData',[point1.y,point3.y],'XData',[point1.x,point3.x],'ZData',[point1.z,point3.z]);
set(L3,'YData',[point2.y,point4.y],'XData',[point2.x,point4.x],'ZData',[point2.z,point4.z]);
set(L4,'YData',[point3.y,point4.y],'XData',[point3.x,point4.x],'ZData',[point3.z,point4.z]);
set(L5,'YData',[point3.y,point5.y],'XData',[point3.x,point5.x],'ZData',[point3.z,point5.z]);
set(L6,'YData',[point4.y,point5.y],'XData',[point4.x,point5.x],'ZData',[point4.z,point5.z]);
plot3(p.base(1),p.base(2),p.base(3),'ob');
    pause(0.1);
end
pause(1);

p.desired = [1000, -500, 100];
plot3(p.desired(1),p.desired(2),p.desired(3),'or');
y=-1500:10:-500+10;
for i=1:1:numel(y)-1
    p.desired = [1000, y(i), 100];

q.abs_d(1) = acot((-p.desired(2) + x_bl*sin(q.base(3) - y_bl*cos(q.base(3))))/(p.base(3)-d_ancher(1)));
q.abs_d(2) = acot((-p.desired(2) + x_bl*sin(q.base(3) - y_bl*cos(q.base(3))))/(p.base(3)-d_ancher(2)));

l.ddesired(1) = sqrt((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)-x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);
l.ddesired(2) = sqrt((p.desired(1)-W_ancher+x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))^2+(p.desired(2)+x_bl*sin(q.base(3))+y_bl*cos(q.base(3)))^2);

alpha_d = acos((p.desired(1)-x_bl*cos(q.base(3))-y_bl*sin(q.base(3)))/l.ddesired(1));
beta_d = acos((W_ancher-p.desired(1)-x_bl*cos(q.base(3))+y_bl*sin(q.base(3)))/l.ddesired(2));

l.desired(1) = sqrt((p.base(3)-d_ancher(1))^2 + l.ddesired(1)^2);
l.desired(2) = sqrt((p.base(3)-d_ancher(2))^2 + l.ddesired(2)^2);

q.motor_d(1) = (l.desired(1) - L_rope(1))/R_sheeve;
q.motor_d(2) = (L_rope(2) - l.desired(2))/R_sheeve;

    
    % position estimation
l_rope = [L_rope(1) + R_sheeve*q.motor_d(1), L_rope(2) - R_sheeve*q.motor_d(2)];
alpha = alpha_d;
beta = beta_d;
q.abs(1) = q.abs_d(1);
q.abs(2) = q.abs_d(2);

h1 = (p.base(3)-d_ancher(1))*cot(q.abs(1));
h2 = (p.base(3)-d_ancher(2))*cot(q.abs(2));

% p.base(1) = h1*cot(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
% p.base(2) = -h1 + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));
p.base(3) = p.base(3);

p.base(1) = l.ddesired(1)*cos(alpha) + x_bl*cos(q.base(3)) + y_bl*sin(q.base(3));
p.base(2) = -l.ddesired(1)*sin(alpha) + x_bl*sin(q.base(3)) - y_bl*cos(q.base(3));
        
    % visualization
hold on
point1.x = 0;point1.y = 0;point1.z = d_ancher(1);
point2.x = W_ancher;point2.y = 0;point2.z = d_ancher(2);
point3.x = l.ddesired(1)*cos(alpha);point3.y = -l.ddesired(1)*sin(alpha);point3.z = p.base(3);
point4.x = W_ancher - l.ddesired(2)*cos(beta);point4.y = -l.ddesired(2)*sin(beta);point4.z = p.base(3);
point5.x = p.base(1);point5.y = p.base(2);point5.z = p.base(3);
set(L1,'YData',[point1.y,point2.y],'XData',[point1.x,point2.x],'ZData',[point1.z,point2.z]);
set(L2,'YData',[point1.y,point3.y],'XData',[point1.x,point3.x],'ZData',[point1.z,point3.z]);
set(L3,'YData',[point2.y,point4.y],'XData',[point2.x,point4.x],'ZData',[point2.z,point4.z]);
set(L4,'YData',[point3.y,point4.y],'XData',[point3.x,point4.x],'ZData',[point3.z,point4.z]);
set(L5,'YData',[point3.y,point5.y],'XData',[point3.x,point5.x],'ZData',[point3.z,point5.z]);
set(L6,'YData',[point4.y,point5.y],'XData',[point4.x,point5.x],'ZData',[point4.z,point5.z]);
plot3(p.base(1),p.base(2),p.base(3),'ob');
    pause(0.1);
end
pause(1);


