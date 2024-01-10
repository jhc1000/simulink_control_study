close all;
clear all;
clc;
syms m1 m2 m3 l1 l2 l3 q1 q2 q3 dotq1 dotq2 dotq3
% m1 = 30; m2 = 17.4; m3 = 4.8; l1 = 0.66; l2 = 0.43; l3 = 0.43; %PUMA
% q1 = u(1); q2 = u(2); q3 = u(3);
% alpha = 0.5;
q = [q1; q2; q3];
dotq = [dotq1; dotq2; dotq3];
n = 3;
d(1,1) = 1/3*m2*l2^2*cos(q2)^2 +1/3*m3*l3^2*(cos(q2+q3))^2 +m3*l2*l3*cos(q2)*cos(q2+q3) +m3*l2^2*cos(q2)^2;
d(1,2) = 0;
d(1,3) = 0;
d(2,1) = 0;
d(2,2) = 1/3*m2*l2^2+1/3*m3*l3^2+m3*l2^2+m3*l2*l3*cos(q3);
d(2,3) = 1/3*m3*l3^2+1/2*m3*l2*l3*cos(q3);
d(3,1) = 0;
d(3,2) = d(2,3);
d(3,3) = 1/3*m3*l3^2;
D = [d(1,1) d(1,2) d(1,3); d(2,1) d(2,2) d(2,3); d(3,1) d(3,2) d(3,3)];

for k = 1:n
    for j = 1:n
        for i = 1:n
            c(i,j,k) = 1/2*(diff(d(k,j),q(i))+diff(d(k,i),q(j))-diff(d(i,j),q(k)));
        end
    end
end

for k = 1:n
    for j = 1:n
        C(k,j) = c(1,j,k)*dotq(1) + c(2,j,k)*dotq(2) + c(3,j,k)*dotq(3);
    end
end
D
C
h_hat=C*dotq

h1 = (-2/3*m2*l2^2*sin(q2)*cos(q2)-2/3*m3*l3^2*cos(q2+q3)*sin(q2+q3)-m3*l2*l3*(cos(q2)*sin(q2+q3)+sin(q2)*cos(q2+q3)) ...
    -2*m3*l2^2*sin(q2)*cos(q2))*dotq1*dotq2-(2/3*m3+l3^2*sin(q2+q3)*cos(q2+q3)+m3*l2*l3*cos(q2)*sin(q2+q3))*dotq1*dotq3;
h2 = (1/3*m3*l2^2*sin(q2)*cos(q2)+1/3*m3*l3^2*cos(q2+q3)*sin(q2+q3)+1/2*m3*l2*l3*(cos(q2)*sin(q2+q3)+sin(q2)*cos(q2+q3)) ...
    +m3*l2^2*sin(q2)*cos(q2))*dotq1^2 - m3*l2*l3*sin(q3)*dotq2*dotq3 - 1/2*m3*l2*l3*sin(q3)*dotq2^2;
h3 = (1/3*m3*l3^2*sin(q2+q3)*cos(q2+q3)+1/2*m3*l2*l3*cos(q2)*sin(q2+q3))*dotq1^2 +(1/2*m3*l2*l3*sin(q3))*dotq2^2;
h = [h1;h2;h3];

for k = 1:n
    for j = 1:n
        D_dot(k,j) = diff(D(k,j),q(1))*dotq(1)+diff(D(k,j),q(2))*dotq(2)+diff(D(k,j),q(3))*dotq(3);
    end
end
D_dot
N=D_dot-2.*C
N+transpose(N)