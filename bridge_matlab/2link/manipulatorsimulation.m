close all
clear all
clc

% Parameter
q.c1 = [0];
q.c2 = [0];
l1 = 200;
l2 = 200;

% Forward Kinematics
p = ForwardKinematics(q);

% Inverse Kinematics
q = InverseKinematics(p);

% ForwardKinematics Simulation
figure;
h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
xlim(pt,[-500 500]);ylim(pt,'manual');ylim(pt,[-500 500]);zlim(pt,'manual');
zlim(pt,[-500 500]);axis;grid on;view(0,90);
title('Wheeleg ForwardKinematics Simulation')
for i = 1:1
point1.x{i} = p.b_c1{i}(1);point1.y{i} = p.b_c1{i}(2);point1.z{i} = p.b_c1{i}(3);
point2.x{i} = p.b_c2{i}(1);point2.y{i} = p.b_c2{i}(2);point2.z{i} = p.b_c2{i}(3);
point3.x{i} = p.b_c3{i}(1);point3.y{i} = p.b_c3{i}(2);point3.z{i} = p.b_c3{i}(3);
L1{i} = line([point1.x{i}, point2.x{i}],[point1.y{i},point2.y{i}],[point1.z{i},point2.z{i}],'Color','r','LineWidth',2);
L2{i} = line([point2.x{i}, point3.x{i}],[point2.y{i},point3.y{i}],[point2.z{i},point3.z{i}],'Color','b','LineWidth',2);
end
pause(1);

% ForwardKinematics Simulation
for theta2 = 0:0.1:(pi/2)
    q.c2 = [theta2];
    p = ForwardKinematics(q); 
    hold on;
    for i =1:1
    point1.x{i} = p.b_c1{i}(1);point1.y{i} = p.b_c1{i}(2);point1.z{i} = p.b_c1{i}(3);
    point2.x{i} = p.b_c2{i}(1);point2.y{i} = p.b_c2{i}(2);point2.z{i} = p.b_c2{i}(3);
    point3.x{i} = p.b_c3{i}(1);point3.y{i} = p.b_c3{i}(2);point3.z{i} = p.b_c3{i}(3);
    set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
    set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
    plot3(point3.x{i},point3.y{i},point3.z{i},'ok');
    end
    pause(0.1);
end
pause(1);

for theta1 = 0:0.1:(pi/2)
    q.c1 = [theta1];
    p = ForwardKinematics(q); 
    hold on;
    for i =1:1
    point1.x{i} = p.b_c1{i}(1);point1.y{i} = p.b_c1{i}(2);point1.z{i} = p.b_c1{i}(3);
    point2.x{i} = p.b_c2{i}(1);point2.y{i} = p.b_c2{i}(2);point2.z{i} = p.b_c2{i}(3);
    point3.x{i} = p.b_c3{i}(1);point3.y{i} = p.b_c3{i}(2);point3.z{i} = p.b_c3{i}(3);
    set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
    set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
    plot3(point3.x{i},point3.y{i},point3.z{i},'ok');
    end
    pause(0.1);
end
pause(1);

% InverseKinematics Simulation
pin.b_c3=p.b_c3;
figure;
h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
xlim(pt,[-500 500]);ylim(pt,'manual');ylim(pt,[-500 500]);zlim(pt,'manual');
zlim(pt,[-500 500]);axis;grid on;view(0,90);
title('Wheeleg InverseKinematics Simulation')
qin = InverseKinematics(pin);
pin2 = ForwardKinematics(qin); 
for i = 1:1
point1.x{i} = pin2.b_c1{i}(1);point1.y{i} = pin2.b_c1{i}(2);point1.z{i} = pin2.b_c1{i}(3);
point2.x{i} = pin2.b_c2{i}(1);point2.y{i} = pin2.b_c2{i}(2);point2.z{i} = pin2.b_c2{i}(3);
point3.x{i} = pin2.b_c3{i}(1);point3.y{i} = pin2.b_c3{i}(2);point3.z{i} = pin2.b_c3{i}(3);
L1{i} = line([point1.x{i}, point2.x{i}],[point1.y{i},point2.y{i}],[point1.z{i},point2.z{i}],'Color','r','LineWidth',2);
L2{i} = line([point2.x{i}, point3.x{i}],[point2.y{i},point3.y{i}],[point2.z{i},point3.z{i}],'Color','b','LineWidth',2);
end
pause(1);

dt = 0.1;
t = 0:dt:10;

xc = 100;
yc = 200;
r = 100;

x = xc + r*cos(pi/5*t);
y = yc + r*sin(pi/5*t);

hold on
for k = 1:length(t)
    pin.b_c3{1} = [x(k); y(k); 0];
    qin = InverseKinematics(pin);
    pin2 = ForwardKinematics(qin); 
    for i = 1:1
    point1.x{i} = pin2.b_c1{i}(1);point1.y{i} = pin2.b_c1{i}(2);point1.z{i} = pin2.b_c1{i}(3);
    point2.x{i} = pin2.b_c2{i}(1);point2.y{i} = pin2.b_c2{i}(2);point2.z{i} = pin2.b_c2{i}(3);
    point3.x{i} = pin2.b_c3{i}(1);point3.y{i} = pin2.b_c3{i}(2);point3.z{i} = pin2.b_c3{i}(3);
    set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
    set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
    plot3(point3.x{i},point3.y{i},point3.z{i},'ok');
    end
    pause(dt);
end


