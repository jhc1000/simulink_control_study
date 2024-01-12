function [A, B] = GetPos(dt)

persistent Velp Posp

if isempty(Posp)
    Posp = 0;
    Velp = 80;
end

w = 0 + 5*randn;
v = 0 + 5*randn;

z = Posp + Velp*dt + v;
A = z;

Posp = z-v;
Velp = 80+w;
B = Velp;