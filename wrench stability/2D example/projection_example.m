% 필요한 패키지를 추가합니다.
% addpath(genpath('path_to_mpt3'));  % MPT3 툴박스의 경로를 추가합니다.

% 6D 렌치 폴리토프 생성
H = [randn(10, 6); -randn(10, 6)];  % 랜덤한 H-표현 행렬
b = rand(20, 1);  % 랜덤한 b-벡터

% 폴리토프 생성
WrenchPolytope = Polyhedron('A', H, 'b', b);

% 프로젝션 차원
projection_dims_3d = [1, 2, 3];
projection_dims_2d = [2, 3];  % 예시: Fy와 Fz 평면으로 투영

% 초기 폴리토프 프로젝션
ProjectedPolytope3D = WrenchPolytope.projection(projection_dims_3d);
ProjectedPolytope2D = WrenchPolytope.projection(projection_dims_2d);

% 시각화 설정
fig = figure;

% 3D 프로젝션 서브플롯
subplot(1, 2, 1);
hold on;
grid on;
axis equal;
view(3);
xlabel('F_x [N]');
ylabel('F_y [N]');
zlabel('F_z [N]');
title('Projected 3D Wrench Polytope');
ProjectedPolytope3D.plot('color', 'blue', 'alpha', 0.5);

% 2D 프로젝션 서브플롯
subplot(1, 2, 2);
hold on;
grid on;
axis equal;
xlabel('F_y [N]');
ylabel('F_z [N]');
title('Projected 2D Wrench Polytope');
ProjectedPolytope2D.plot('color', 'green', 'alpha', 0.5);

% 반복적으로 제약 조건을 적용
for iter = 1:10
    % 새로운 제약 조건 추가 (여기서는 랜덤 제약 조건 예시)
    new_constraint_H = randn(1, 6);
    new_constraint_b = rand;
    
    % 기존 폴리토프에 새로운 제약 조건 적용
    WrenchPolytope = Polyhedron('A', [WrenchPolytope.A; new_constraint_H], 'b', [WrenchPolytope.b; new_constraint_b]);
    
    % 3D로 프로젝션 갱신
    ProjectedPolytope3D = WrenchPolytope.projection(projection_dims_3d);
    
    % 2D로 프로젝션 갱신
    ProjectedPolytope2D = WrenchPolytope.projection(projection_dims_2d);
    
    % 기존 플롯을 지우고 새로운 폴리토프를 플로팅
    subplot(1, 2, 1);
    cla;
    ProjectedPolytope3D.plot('color', 'blue', 'alpha', 0.5);
    title('Projected 3D Wrench Polytope');
    xlabel('F_x [N]');
    ylabel('F_y [N]');
    zlabel('F_z [N]');
    axis equal;
    view(3);
    
    subplot(1, 2, 2);
    cla;
    ProjectedPolytope2D.plot('color', 'green', 'alpha', 0.5);
    title('Projected 2D Wrench Polytope');
    xlabel('F_y [N]');
    ylabel('F_z [N]');
    axis equal;

    % 애니메이션 효과를 위한 일시 정지
    pause(0.5);
end
