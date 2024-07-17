close all
clear all
clc

% 초기 사각형을 정의 (4개의 꼭지점)
initial_rectangle = [5 5;5 -5;-5 -5;-5 5];

% 초기 작은 사각형의 범위 설정
x_min_initial = -3;
x_max_initial = 3;
y_min_initial = -3;
y_max_initial = 3;

% 초기 작은 사각형 내부의 임의의 점들 생성
num_initial_points = 4;
initial_range_points = [x_min_initial + (x_max_initial - x_min_initial) * rand(num_initial_points, 1), ...
                        y_min_initial + (y_max_initial - y_min_initial) * rand(num_initial_points, 1)];

% 예제 실행
epsilon = 0.001;  % Area difference tolerance
outer_points = initial_rectangle;  % Initial outer region points (rectangle vertices)
inner_points = initial_range_points;  % Initial inner region points

% 반복 횟수 제한 (보통 무한 반복을 막기 위해 설정)
max_iterations = 1000;
iteration = 1;

% 최적화 옵션 설정
options = optimoptions('linprog', 'Display', 'iter');

while iteration <= max_iterations
    % 확장된 점들의 볼록 다각형 (outer region) 계산
    k_outer = boundary(outer_points(:,1), outer_points(:,2));
    outer_polygon = polyshape(outer_points(k_outer, 1), outer_points(k_outer, 2));

    % 초기 작은 사각형의 볼록 다각형 (inner region) 계산
    k_inner = boundary(inner_points(:,1), inner_points(:,2));
    inner_polygon = polyshape(inner_points(k_inner, 1), inner_points(k_inner, 2));

    % 두 영역의 면적 계산
    area_outer = area(outer_polygon);
    area_inner = area(inner_polygon);

    % 면적 차이 계산
    area_difference = abs(area_outer - area_inner);

    % 면적 차이가 epsilon 이하거나 inner region이 outer region을 넘어가면 종료
    if area_difference < epsilon || area_inner > area_outer
        break;
    end

    % inner_points의 평균 계산
    mean_inner = mean(inner_points);

    % outer_points를 inner_points 방향으로 축소
    direction_vectors_outer = outer_points - mean_inner;
    outer_points = outer_points - direction_vectors_outer * 0.01;  % Shrink factor towards inner region centroid

    % 선형 프로그램을 사용하여 inner_points 확장
    num_points_inner = size(inner_points, 1);
    f_inner = ones(num_points_inner, 1);  % 최소화를 위해 양수로 설정
    A_inner = [];
    b_inner = [];
    Aeq_inner = ones(1, num_points_inner);  % 가중치의 합이 1이 되도록 설정
    beq_inner = 1;
    lb_inner = zeros(num_points_inner, 1);  % 가중치의 하한
    ub_inner = ones(num_points_inner, 1);  % 가중치의 상한

    % 선형 프로그래밍을 통해 가중치 계산
    [weights_inner, ~, exitflag_inner] = linprog(f_inner, A_inner, b_inner, Aeq_inner, beq_inner, lb_inner, ub_inner, options);

    if exitflag_inner == 1  % 해결책이 발견된 경우
        weighted_sum_inner = inner_points' * weights_inner;
        new_point_inner = weighted_sum_inner / sum(weights_inner);

        % 각 점을 새로운 점 방향으로 확장
        direction_vectors_inner = inner_points - mean(inner_points);
        inner_points = inner_points + direction_vectors_inner * 0.01;  % Expansion factor for inner region
    else
        disp('Linear programming did not converge to a solution for inner region.');
        break;  % 선형 프로그램 최적화가 수렴하지 않으면 종료
    end

    iteration = iteration + 1;
end

% 최종 결과 플롯
figure;
hold on;

% 초기 사각형의 점들 플롯 (점으로 표시)
scatter(initial_rectangle(:, 1), initial_rectangle(:, 2), 100, 'k', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Original Rectangle Points');

% 초기 사각형의 점들을 연결하여 dotted line으로 플롯
initial_rectangle_connected = [initial_rectangle; initial_rectangle(1,:)];
plot(initial_rectangle_connected(:,1), initial_rectangle_connected(:,2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Initial Rectangle');

% 초기 outer region의 점들 플롯
scatter(outer_points(:, 1), outer_points(:, 2), 100, 'b', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Shrunk Outer Points');

% 초기 inner region의 점들 플롯
scatter(initial_range_points(:, 1), initial_range_points(:, 2), 100, 'g', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Initial Inner Points');

% 최종 inner region의 점들 플롯 (점으로 표시)
scatter(inner_points(:, 1), inner_points(:, 2), 100, 'r', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Final Expanded Inner Points');

% 각 점들을 연결하는 경계선 플롯
k_outer = boundary(outer_points(:,1), outer_points(:,2));
plot(outer_points(k_outer,1), outer_points(k_outer,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Boundary of Shrunk Outer Region');

k_inner_initial = boundary(initial_range_points(:,1), initial_range_points(:,2));
plot(initial_range_points(k_inner_initial,1), initial_range_points(k_inner_initial,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Boundary of Initial Inner Region');

k_inner_final = boundary(inner_points(:,1), inner_points(:,2));
plot(inner_points(k_inner_final,1), inner_points(k_inner_final,2), 'r-', 'LineWidth', 2, 'DisplayName', 'Boundary of Final Expanded Inner Region');

hold off;
title('Comparison of Shrunk Outer and Expanded Inner Regions');
xlabel('X');
ylabel('Y');
legend show;
grid on;

disp(['Final area difference: ', num2str(area_difference)]);
