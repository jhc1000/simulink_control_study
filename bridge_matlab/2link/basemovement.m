close all
clear all
clc

% Parameter
q.b = [0];
q.p1 = [0];
q.p2 = [0];
l1 = 150;
l2 = 200;
l3 = 300;
lb = 100;

pc = [100, 150]';
p1 = [0, 0]';
p2 = [l3, 0]';

q.p1 = acos((pc(1)-p1(1)-lb*cos(q.b))/l1);
q.p2 = acos((pc(1)-p2(1)+lb*cos(q.b))/l2);

pc(2) = p2(2) + l2*sin(q.p2) - lb*sin(q.b);
pa = p1 + [l1*cos(q.p1), l1*sin(q.p1)]';
pb = pc + [l1*cos(q.b), l1*sin(q.b)]';


point1.x = p1(1);point1.y= p1(2);
point2.x = pa(1);point2.y= pa(2);
point3.x = pc(1);point3.y= pc(2);
point4.x = pb(1);point4.y= pb(2);
point5.x = p2(1);point5.y= p2(2);
L1 = line([point1.x, point2.x],[point1.y,point2.y],[0,0],'Color','r','LineWidth',2);
L2 = line([point2.x, point4.x],[point2.y,point4.y],[0,0],'Color','b','LineWidth',2);
L3 = line([point4.x, point5.x],[point4.y,point5.y],[0,0],'Color','g','LineWidth',2);
pause(1);

dt = 0.1;
t = 0:dt:10;

xc = 150;
yc = 175;
r = 20;

x = xc + r*cos(pi/5*t);

hold on
for k = 1:length(t)
  
    pc(1) = x(k);
    q.b = 0;

    q.p1 = acos((pc(1)-p1(1)-lb*cos(q.b))/l1);
    q.p2 = acos((pc(1)-p2(1)+lb*cos(q.b))/l2);

    pc(2) = p2(2) + l1*sin(q.p1) + lb*sin(q.b);
    
    pa = p1 + [l1*cos(q.p1), l1*sin(q.p1)]';
    pb = pc + [lb*cos(q.b), lb*sin(q.b)]';


    point1.x = p1(1);point1.y= p1(2);
    point2.x = pa(1);point2.y= pa(2);
    point3.x = pc(1);point3.y= pc(2);
    point4.x = pb(1);point4.y= pb(2);
    point5.x = p2(1);point5.y= p2(2);
    set(L1,'ZData',[0,0],'YData',[point1.y,point2.y],'XData',[point1.x, point2.x]);
    set(L2,'ZData',[0,0],'YData',[point2.y,point4.y],'XData',[point2.x, point4.x]);
    set(L3,'ZData',[0,0],'YData',[point4.y,point5.y],'XData',[point4.x, point5.x]);
    plot(pc(1),pc(2),'ok');
    pause(dt);
end



