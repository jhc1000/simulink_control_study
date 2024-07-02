% Define the vertices of the two convex hulls
P1 = [0 0 0; 1 0 0; 0 1 0; 0 0 1];
P2 = [0 0 0; 1 1 0; 0 1 1; 1 0 1];

% Initialize an empty array to store the Minkowski sum points
M = [];

% Compute the Minkowski sum
for i = 1:size(P1, 1)
    for j = 1:size(P2, 1)
        M = [M; P1(i, :) + P2(j, :)];
    end
end

% Compute the convex hull of the Minkowski sum
K = convhull(M(:, 1), M(:, 2), M(:, 3));

% Plot the result
figure
trisurf(K, M(:, 1), M(:, 2), M(:, 3), 'FaceColor', 'cyan')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Convex Hull of the Minkowski Sum')
axis equal
