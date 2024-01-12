function [T] = TransformationMatrix(R,p)
    T = [R, p
        [0 0 0] , 1];
end

