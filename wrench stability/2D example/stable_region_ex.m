close all
clear all
clc

% 초기 사각형을 정의 (4개의 꼭지점)
initial_rectangle = [1 1; 1 5; 5 5; 5 1];

% 초기 작은 사각형의 범위 설정
x_min_initial = 2;
x_max_initial = 3;
y_min_initial = 2;
y_max_initial = 3;

% 초기 작은 사각형 내부의 임의의 점들 생성
num_initial_points = 4;
initial_range_points = [x_min_initial + (x_max_initial - x_min_initial) * rand(num_initial_points, 1), ...
                        y_min_initial + (y_max_initial - y_min_initial) * rand(num_initial_points, 1)];

% 예제 실행
num_iterations = 10;  % 반복 횟수 설정
expansion_factor = 0.2;  % 사각형 확대 비율
target_points = 20;  % 목표 점의 개수
[result, initial_points] = iterative_expansion(initial_range_points, num_iterations, expansion_factor, target_points);
disp('Expanded set of points:');
disp(result);

% 점들과 중심을 플롯
centroid = mean(result);  % 중심 계산

figure;
hold on;

% 초기 사각형의 점들 플롯
scatter(initial_rectangle(:, 1), initial_rectangle(:, 2), 100, 'k', 'filled', 'DisplayName', 'Original Rectangle Points', 'MarkerEdgeColor', 'k');

% 초기 사각형의 꼭지점들을 점선으로 연결
plot([initial_rectangle(:, 1); initial_rectangle(1, 1)], [initial_rectangle(:, 2); initial_rectangle(1, 2)], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Initial Rectangle Dotted Lines');

% 초기 작은 사각형의 점들 플롯 (다른 색상)
scatter(initial_points(:, 1), initial_points(:, 2), 100, 'b', 'filled', 'DisplayName', 'Initial Small Range Points', 'MarkerEdgeColor', 'k');

% 최종 확장된 점들의 볼록 다각형 (convex hull) 플롯
k = boundary(result(:,1), result(:,2));
plot(result(k,1), result(k,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Convex Hull of Expanded Points');

% 중심 플롯
scatter(centroid(1), centroid(2), 300, 'm', 'filled', 'DisplayName', 'Centroid');

% 확장된 점들 플롯
scatter(result(:, 1), result(:, 2), 100, 'r', 'filled', 'DisplayName', 'Final Expanded Points', 'MarkerEdgeColor', 'k');

hold off;
title('Comparison of Original and Expanded Set of Points');
xlabel('X');
ylabel('Y');
legend show;
grid on;

% Iterative expansion을 사용하여 사각형을 이루는 점들을 늘리기
function [expanded_points, initial_points] = iterative_expansion(points, num_iterations, expansion_factor, target_points)
    initial_points = points;
    
    for iter = 1:num_iterations
        if size(points, 1) >= target_points
            break;
        end
        
        % 선형 프로그램을 사용하여 점들을 확장
        centroid = mean(points);  % 현재 점들의 중심 계산
        num_points = size(points, 1);
        f = -ones(num_points, 1);  % 최대화를 위해 음수로 설정
        A = [];
        b = [];
        Aeq = ones(1, num_points);  % 가중치의 합이 1이 되도록 설정
        beq = 1;
        lb = zeros(num_points, 1);  % 가중치의 하한
        ub = ones(num_points, 1);  % 가중치의 상한
        
        % 선형 프로그램을 통해 가중치 계산
        [weights, ~, exitflag] = linprog(f, A, b, Aeq, beq, lb, ub);
        
        if exitflag == 1  % 해결책이 발견된 경우
            weighted_sum = points' * weights;
            new_point = weighted_sum / sum(weights);
            
            % 각 점을 새로운 점 방향으로 확장
            direction_vectors = points - centroid;
            new_points = points + direction_vectors * expansion_factor;
        else
            error('Linear programming did not converge to a solution.');
        end
        
        points = [points; new_points];  % 새로운 점들을 기존 점들에 추가
        points = unique(points, 'rows');  % 중복 점 제거
    end
    
    % 목표 점의 개수까지 점을 채우고 나서 마지막 결과 반환
    if size(points, 1) > target_points
        expanded_points = points(1:target_points, :);
    else
        expanded_points = points;
    end
end
