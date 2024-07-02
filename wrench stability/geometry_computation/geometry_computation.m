classdef geometry_computation
    properties
    end
    
    methods
        function obj = geometry_computation(obj)
        end
    end
    methods(Static)
        function M = minkowskiSum(P1,P2)
            % Initialize an empty array to store the Minkowski sum points
            M = [];
            
            % Compute the Minkowski sum
            for i = 1:size(P1, 1)
                for j = 1:size(P2, 1)
                    M = [M; P1(i, :) + P2(j, :)];
                end
            end
            
            % % Compute the convex hull of the Minkowski sum
            % K = convhull(M(:, 1), M(:, 2), M(:, 3));
        end
    end
end

