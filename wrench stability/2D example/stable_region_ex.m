close all
clear all
clc

% 초기 사각형을 정의 (4개의 꼭지점)
rectangle_points = [1 1; 1 5; 5 5; 5 1];

% 초기 사각형의 점들
initial_rectangle = rectangle_points;

% 예제 실행
result = iterative_projection(initial_rectangle, 2);  % 예시로 2번만 반복하도록 설정
disp('Reduced set of points:');
disp(result);

% 줄어든 점들과 중심을 플롯
centroid = mean(result);  % 중심 계산

figure;
hold on;

% 초기 사각형의 점들 플롯
scatter(initial_rectangle(:, 1), initial_rectangle(:, 2), 'filled', 'DisplayName', 'Original Rectangle Points');
plot([initial_rectangle(:, 1); initial_rectangle(1, 1)], [initial_rectangle(:, 2); initial_rectangle(1, 2)], 'k--');

% 줄어든 지역의 점들 플롯
scatter(result(:, 1), result(:, 2), 'filled', 'DisplayName', 'Reduced Points');
plot([result(:, 1); result(1, 1)], [result(:, 2); result(1, 2)], 'r--');

% 중심 플롯
scatter(centroid(1), centroid(2), 100, 'm', 'filled', 'DisplayName', 'Centroid');

% 초기 사각형 내부의 점들을 확인하여 플롯
inside_points = points_inside_rectangle(initial_rectangle, result);
scatter(inside_points(:, 1), inside_points(:, 2), 100, 'g', 'filled', 'DisplayName', 'Points Inside Rectangle');

hold off;
title('Comparison of Original and Reduced Set of Points');
xlabel('X');
ylabel('Y');
legend show;
grid on;

% Linear programming 문제를 푸는 함수
function [centroid, weights] = minimize_points(points)
    num_points = size(points, 1);
    f = ones(num_points, 1);  % minimize f^T * x (sum of distances)
    Aeq = ones(1, num_points);  % sum of weights must be 1
    beq = 1;
    lb = zeros(num_points, 1);  % weights are between 0 and 1
    ub = ones(num_points, 1);
    
    % Objective function to minimize (sum of distances)
    distance_fun = @(x) sum(pdist2(points, points * x) .* x');
    
    % Solve linear programming problem
    options = optimoptions('linprog', 'Algorithm', 'interior-point', 'Display', 'off');
    [weights, ~, exitflag] = linprog(f, [], [], Aeq, beq, lb, ub, options);
    
    if exitflag == 1  % Check if solution is found
        weighted_sum = points' * weights;
        centroid = weighted_sum / sum(weights);
    else
        error('Linear programming did not converge to a solution.');
    end
end

% Iterative projection을 사용하여 사각형을 이루는 점들을 줄이기
function reduced_points = iterative_projection(points, num_iterations)
    for iter = 1:num_iterations
        % Calculate distances from each point to all other points
        distances = pdist2(points, points);
        
        % Find the index of the point farthest from any other point
        [~, farthest_index] = max(sum(distances, 2));
        
        % Remove the farthest point
        points(farthest_index, :) = [];
    end
    reduced_points = points;
end

% 사각형 내부에 있는 점들을 확인하는 함수
function inside_points = points_inside_rectangle(rectangle_points, points)
    % 사각형의 점 좌표
    x = rectangle_points(:, 1);
    y = rectangle_points(:, 2);
    
    % 사각형의 경계를 정의
    rect_x = [x; x(1)];
    rect_y = [y; y(1)];
    
    % 점들이 사각형 내부에 있는지 확인
    inside = inpolygon(points(:, 1), points(:, 2), rect_x, rect_y);
    
    % 사각형 내부에 있는 점들 반환
    inside_points = points(inside, :);
end
