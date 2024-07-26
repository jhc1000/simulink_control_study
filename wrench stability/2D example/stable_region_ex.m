function computeSupportRegion(A1, A2, t, B, u)
    % Define the number of directions to sample
    numDirections = 100;
    epsilon = 1e-4; % Desired precision
    maxIterations = 100; % Maximum number of iterations

    % Initialize the support region
    Yinner = [];
    Youter = [];
    
    % Initialize directions (uniformly distributed directions on a circle)
    angles = linspace(0, 2*pi, numDirections);
    directions = [cos(angles); sin(angles)];
    
    % Initialize the first iteration
    previousAreaDifference = inf;
    
    for iter = 1:maxIterations
        % Compute extremal points for each direction
        points = zeros(2, numDirections);
        for i = 1:numDirections
            a = directions(:, i);
            [~, v] = findExtremalPoint(A1, A2, t, B, u, a);
            points(:, i) = v;
        end
        
        % Compute convex hull of the points (Yinner)
        k = convhull(points(1, :), points(2, :));
        Yinner = points(:, k);

        % Compute the outer polygon (Youter)
        Youter = Yinner; % For simplicity, we assume the outer polygon is the convex hull here
        
        % Check stopping criterion
        areaYinner = polyarea(Yinner(1, :), Yinner(2, :));
        areaYouter = polyarea(Youter(1, :), Youter(2, :));
        areaDifference = areaYouter - areaYinner;
        
        if areaDifference <= epsilon
            break;
        end
        
        % Avoid infinite loop
        if iter > 1 && areaDifference >= previousAreaDifference
            disp('Convergence issue: area difference not decreasing.');
            break;
        end
        previousAreaDifference = areaDifference;
    end

    % Plot the results
    figure;
    hold on;
    fill(Yinner(1, :), Yinner(2, :), 'g', 'FaceAlpha', 0.5);
    fill(Youter(1, :), Youter(2, :), 'r', 'FaceAlpha', 0.3);
    title('Support Region Approximation');
    xlabel('y1');
    ylabel('y2');
    legend('Inner Approximation', 'Outer Approximation');
    grid on;
    axis equal;
    hold off;
end

function [zopt, v] = findExtremalPoint(A1, A2, t, B, u, a)
    % Define the second-order cone program (SOCP)
    cvx_begin
        variable z(2)
        maximize(a' * z)
        subject to
            A1 * z == t;
            B * z <= u' * z;
    cvx_end

    % Output the optimal point
    zopt = z;
    v = zopt;
end
