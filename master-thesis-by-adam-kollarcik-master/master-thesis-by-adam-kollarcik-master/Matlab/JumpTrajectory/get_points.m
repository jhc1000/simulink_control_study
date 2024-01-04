function [x0,x1,x12,x2,x23,x14,x4,x34,x3,x45,B] = get_points(th0,th1,th2,th3,th4,r,r3,ang_3,xW,yW,l1c,l11,l2c,l4c,l3,a_body,b_body)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here



T0 = trans_matrix2D([xW;yW],0);
T01c =  T0*trans_matrix2D([0;0],th1+pi/2)*trans_matrix2D([l1c;0],0);
T012 = T01c * trans_matrix2D([l11-l1c;0],0);
T014 = T01c * trans_matrix2D([l1c;0],0);
T02c = T012*trans_matrix2D([0;0],th2)*trans_matrix2D([l2c;0],0);
T023 = T02c*trans_matrix2D([l2c;0],0);
T04c = T014*trans_matrix2D([0;0],th4)*trans_matrix2D([l4c;0],0);
T04 = T04c *trans_matrix2D([l4c;0],0);
T03rot =  T023*trans_matrix2D([0;0],th3);
T03c = T03rot *trans_matrix2D([0;0],ang_3)*trans_matrix2D([r3;0],0);
T034 = T03rot*trans_matrix2D([l3;0],0);

rectL = sqrt((a_body/2)^2+(b_body/2)^2);
rectAng = atan((a_body/2)/(b_body/2));

TB1 = T03c*trans_matrix2D([0;0],rectAng)*trans_matrix2D([rectL;0],0);
TB2 = T03c*trans_matrix2D([0;0],-pi-rectAng)*trans_matrix2D([rectL;0],0);
TB3 = T03c*trans_matrix2D([0;0],-pi+rectAng)*trans_matrix2D([rectL;0],0);
TB4 = T03c*trans_matrix2D([0;0],-rectAng)*trans_matrix2D([rectL;0],0);


x0 = T0*[0;0;1];
x0 = x0(1:2);
x1 = T01c*[0;0;1];
x1 = x1(1:2);
x12 = T012*[0;0;1];
x12 = x12(1:2);
x2 =T02c*[0;0;1];
x2 = x2(1:2);

x23 = T023*[0;0;1];
x23 = x23(1:2);
x14 = T014*[0;0;1];
x14 = x14(1:2);
x4 =T04c*[0;0;1];
x4 = x4(1:2);
x34 = T034*[0;0;1];
x34 = x34(1:2);
x45 = T04*[0;0;1];
x45 = x45(1:2);
x3 =T03c*[0;0;1];
x3 = x3(1:2);


b1 =TB1*[0;0;1];
b1 = b1(1:2);

b2 =TB2*[0;0;1];
b2 = b2(1:2);

b3 =TB3*[0;0;1];
b3 = b3(1:2);

b4 =TB4*[0;0;1];
b4 = b4(1:2);

B = [b1 b2 b3 b4];




end

