classdef math_tools
    properties

    end

    methods(Static)
        function skv = skew(v)
            % Check if the input vector has 4 elements
            if length(v) == 4
                % Convert homogeneous coordinates to Cartesian coordinates
                v = v(1:3) / v(4);
            end

            % Ensure v is a column vector
            v = v(:);

            % Roll the diagonal matrix and subtract its transpose
            skv = circshift(circshift(diag(v), 1, 2), -1, 1) - circshift(circshift(diag(v), 1, 1), -1, 2);
        end

        function Rot = rpyToRot(roll, pitch, yaw)
            Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            Ry = [cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)];
            Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll);];

            Rot = Rz*Ry*Rx;
        end

        function T = homogeneous_matrix(R, p)
            T = [R p;
                [0 0 0]  1];

        end
        function T = T_m(dx,dy,dz,roll,pitch,yaw)

            p = [dx, dy, dz]';
            R = math_tools.rpyToRot(roll, pitch, yaw);
            T = math_tools.homogeneous_matrix(R, p);
        end
        function T_inv = homog_transform_inverse(T)
            R_inv = T(1:3,1:3)';
            p_inv = -T(1:3,1:3)'*T(1:3,4);
            T_inv = [R_inv p_inv;
                [0 0 0]  1];
        end
        function Ad = adjoint_matrix(R, p)
            Ad = [R zeros(3,3);
                math_tools.skew(p)*R  R];

        end
        function D = hstack(A,B)
            D = horzcat(A,B);
        end
        function D = vstack(A,B)
            D = vertcat(A,B);
        end
        % 다각형의 꼭지점과 특정 점 간의 최소 거리를 계산하는 함수
        function minDist = minDistanceToPolygon3D(com_position_lp_results, target_point)
            import math_tools.*
            % 다각형 꼭지점의 개수 (열 개수)
            n = size(com_position_lp_results, 2);
            % 초기 최소 거리를 큰 값으로 설정
            minDist = inf;

            for i = 1:n
                % 현재 변의 시작점과 끝점 (3x1 벡터로 설정)
                start = com_position_lp_results(:, i);
                if i < n
                    endP = com_position_lp_results(:, i + 1);
                else
                    endP = com_position_lp_results(:, 1); % 마지막 꼭지점의 다음은 첫 꼭지점으로
                end

                % 점과 현재 선분(start-endP) 사이의 거리 계산
                dist = pointToLineSegmentDistance3D(target_point, start, endP);
                % 최소 거리 업데이트
                minDist = min(minDist, dist);
            end
        end

        % 3차원 공간에서 점과 선분 간의 거리를 계산하는 함수
        function dist = pointToLineSegmentDistance3D(point, start, endP)
            % 선분의 벡터 (3x1 벡터로 설정)
            lineVec = endP - start;
            % 시작점과 점(point) 사이의 벡터
            pointVec = point - start;
            % 선분의 길이의 제곱
            lineLen = dot(lineVec, lineVec);

            if lineLen == 0
                % 선분이 점인 경우 (start와 endP가 같은 경우)
                dist = norm(pointVec);
                return;
            end

            % t는 점의 직교 투영 위치를 결정
            t = max(0, min(1, dot(pointVec, lineVec) / lineLen));
            % 투영된 점의 좌표
            projection = start + t * lineVec;
            % 투영된 점과 점 사이의 거리
            dist = norm(projection - point);
        end
    end
end

