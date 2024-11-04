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
        function minDist = minDistanceToPolygon3D(polygonVertices, point)
            import math_tools.*
            % Calculate the minimum distance from the point to the polygon edges
            numVertices = size(polygonVertices, 2);
            minDist = inf;

            % Calculate the normal vector of the polygon's plane
            v1 = polygonVertices(:, 2) - polygonVertices(:, 1);
            v2 = polygonVertices(:, 3) - polygonVertices(:, 1);
            normal = cross(v1, v2);
            normal = normal / norm(normal); % Normalize the normal vector

            % Project the point onto the plane of the polygon
            pointToVertex = point - polygonVertices(:, 1);
            distanceToPlane = dot(pointToVertex, normal);
            projectedPoint = point - distanceToPlane * normal;

            % Loop through each edge to find the minimum distance
            for i = 1:numVertices
                startPoint = polygonVertices(:, i);
                endPoint = polygonVertices(:, mod(i, numVertices) + 1);
                % Calculate the distance from the point to the current edge
                dist = pointToLineSegmentDistance(point, startPoint, endPoint);
                if dist < minDist
                    minDist = dist;
                end
            end

            % Project polygon vertices to 2D on the plane for point-in-polygon check
            projVertices = polygonVertices - (normal * (normal' * polygonVertices));
            projPoint = projectedPoint - (normal * (normal' * projectedPoint));

            % Convert to 2D by discarding the z-component (arbitrary choice based on normal)
            projVertices2D = projVertices(1:2, :);
            projPoint2D = projPoint(1:2);

            % Check if the point is inside the polygon using 2D coordinates
            inPolygon = inpolygon(projPoint2D(1), projPoint2D(2), projVertices2D(1, :), projVertices2D(2, :));

            % If the point is outside, make the minimum distance negative
            if ~inPolygon
                minDist = -minDist;
            end
        end

        function dist = pointToLineSegmentDistance(point, start, endP)
            % Calculate the minimum distance between a point and a line segment in 3D
            lineVec = endP - start;
            pointVec = point - start;
            lineLen = dot(lineVec, lineVec);
            if lineLen == 0
                % Start and end points are the same
                dist = norm(pointVec);
                return;
            end
            % Project point onto the line, clamping to segment endpoints
            t = max(0, min(1, dot(pointVec, lineVec) / lineLen));
            projPoint = start + t * lineVec;
            dist = norm(point - projPoint);
        end
    end
end

