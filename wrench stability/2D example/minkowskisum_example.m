close all
clear all
clc;

figure(1);
Pa = polyshape(rand(3,2));
Pb = polyshape(rand(3,2));
MSumShape = minkowskiSum(Pa,Pb);
plot(Pa)
hold on
plot(Pb)
plot(MSumShape)
hold off

figure(2);
Pa = polyshape([0 0;1 0;1 1;0 1]); %%input vertex of polyshape
plot(Pa)
hold on
MSumShape = minkowskiSum(Pa,Pa);
plot(MSumShape)
hold off
axis equal

figure(3);
t = linspace(0,2*pi)'; t(end) = [];
Pa = polyshape([2*cos(t),sin(t)]);
Pb = polyshape([cos(t),3*sin(t)]);
plot(Pa)
hold on
plot(Pb)
MSumShape = minkowskiSum(Pa,Pb);
plot(MSumShape)
hold off
axis equal

figure(4);
Pa = polyshape([-1 2;0 2;-0.75 2.25;-1 3]);
Pb = polyshape([1 0;2 0;3 1;4 3;2 .3]);
plot(Pa)
hold on
plot(Pb)
MSumShape = minkowskiSum(Pa,Pb);
plot(MSumShape)
hold off

figure(5);
Pa = polyshape(rand(6,2));
Pb = polyshape(rand(6,2));
Psum = minkowskiSum(Pa,Pb);
plot(Pa)
hold on
plot(Pb)
plot(Psum)