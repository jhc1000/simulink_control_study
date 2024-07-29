close all
clear all
clc

% 설정된 매개변수 및 행렬 불러오기
ai = [1.0; 0.1];
% 위에서 제공한 코드가 설정되어 있다고 가정

% 관심 있는 범위 설정 (예: -10에서 10)
x1_range = -10:0.1:10;
x2_range = -10:0.1:10;

% 그리드 생성
[X1, X2] = meshgrid(x1_range, x2_range);
num_points = numel(X1);
X = [X1(:), X2(:)]'; % 2 x num_points 크기의 행렬

% A_ineq와 b_ineq 불러오기 (이미 설정된 상태로 가정)
% 예제:
A_ineq = [1, 2; -1, 2; 1, -2; -1, -2]; % 임의의 A_ineq (사용자의 설정에 맞게 수정)
b_ineq = [10; 10; 10; 10]; % 임의의 b_ineq (사용자의 설정에 맞게 수정)

% 부등식 평가
valid_points = [];
for i = 1:num_points
    x = X(:, i);
    if all(A_ineq * x <= b_ineq)
        valid_points = [valid_points, x];
    end
end

% 플롯
figure;
hold on;
plot(valid_points(1, :), valid_points(2, :), 'bo');
title('Points satisfying A_{ineq} \cdot x \leq b_{ineq}');
xlabel('x1');
ylabel('x2');
grid on;
hold off;
