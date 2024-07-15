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
    end
end

