function R = rot(axis,q)

if axis == 'x'
    R = [1, 0, 0;0, cos(q), -sin(q);0, sin(q), cos(q)];
elseif axis == 'y'
    R = [cos(q), 0, sin(q);0, 1, 0;-sin(q), 0, cos(q)];
elseif axis == 'z'
    R = [cos(q), -sin(q), 0;sin(q), cos(q), 0;0, 0, 1];
end