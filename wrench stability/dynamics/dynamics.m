classdef dynamics
    properties
        grf
        moment
        tension
        mu
    end
    
    methods
        function self = init(~, self)
            self.grf.s_w = zeros(3,4);
            self.tension.s_ej = zeros(3,2);
            self.moment.s_w = zeros(3,4);
            self.moment.s_ej = zeros(3,2);
            self.mu = 0.3;
        end
    end
    methods(Static)
        function self = grf_estimation_static(self, q_base, b_base)
            for i=1:4
                self.moment.s_w(:,i) = cross(self.s_w(:,i),self.grf.s_w(:,i));
            end

            for i=1:2
                self.moment.s_ej(:,i) = cross(self.s_ej(:,i),self.tension.s_ej(:,i));
            end

        
        end
    end
end

