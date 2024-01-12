close all
clear all
clc

m = 10;
b = 3;
k = 5;
a = 2;
w = 4;

syms s

A = [0 1; -k/m -b/m];
B = [0;1/m];
H = [1 0];



G = H*inv(s*eye(2)-A)*B

U = (a*w^2)/(s^2+w^2)

Z = G*U

z = ilaplace(Z)