classdef leg_force_polytope
    properties
        forcePolytope
    end
    
    methods
        function obj = leg_force_polytope(obj, numberOfLegs)
            import polytope.*
            %POLYTOPE 이 클래스의 인스턴스 생성
            %   자세한 설명 위치
            polytope = zeros(numberOfLegs);
            obj.forcePolytope = [];

            for i=1:numberOfLegs
                polytope(i) = polytope();
                obj.forcePolytope = obj.forcePolytope + polytope(i);
            end
        end
        function value = getVertices(obj)
             value = obj.forcePolytope.vertices;
        end
        function value = getHalfspaces(obj)
            value = obj.forcePolytope.halfspaces;
        end
    end
end

