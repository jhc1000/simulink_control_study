close all;
clearvars -except out;
clc;

xd = out.xd;
yd = out.yd;
zd = out.zd;

x = out.x(:,1);
y = out.x(:,2);
z = out.x(:,3);

plot3([0, 0.8], [0, 0.8], [0.5, 0.8], 'Color', 'none')
hold on
grid on
axis image
title('Trajectory PD Control');
xlabel('x axis(m)');ylabel('y axis(m)');zlabel('z axis(m)');

plot3(xd, yd, zd, '--');
plot3(x, y, z, 'b');
