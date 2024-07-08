classdef StableRegionComputation
    properties
        epsilon
        model  % Define model property
    end

    methods
        function obj = StableRegionComputation(epsilon)
            % Constructor
            obj.epsilon = epsilon;
        end

        function obj = init(obj, model)
            % Initialize the model property
            obj.model = model;
        end

        function [cxy_opt, f_opt] = solveLP(~, self, ai, cxy, alpha, R_sb, p, n, mu, tau_min, tau_max, ej, v, t_min, t_max)
            % Solve the LP problem
            % Example implementation
            % Accessing model property
            m = self.model.totalmass;
            g = [-9.8*sin(alpha), 0, -9.8*cos(alpha)].';
            P = [1 0 0; 0 1 0;0 0 0];
            A1 = [];
            A2 = [];
            A3 = [];
            B = [];
            G = [];
            W = [];
            u = [];
            d = [];
            w = [];

            % A1
            for i = 1:size(p, 2)
                A_upperbar(:,:,i) = [eye(3); math_tools.skew(p(:,i))];
                A1 = horzcat(A1, A_upperbar(:,:,i));
            end

            % A2
            A2 = [zeros(3,3); -m.*cross(repmat(g, 1, 3), P.')];

            % A3
            for i = 1:size(ej, 2)
                A_underbar(:,:,i) = [eye(3); math_tools.skew(ej(:,i))];
                A3 = horzcat(A3, A_underbar(:,:,i));
            end

            % u
            u = [-m.*g; zeros(3,1)];

            % B
            b = [([1;0;0]-mu*[0;0;1]).';
                ([0;1;0]-mu*[0;0;1]).';
                -([1;0;0]+mu*[0;0;1]).';
                -([1;0;0]+mu*[0;0;1]).'];
            for i = 1:size(p, 2)
                B = blkdiag(B, b);
            end

            % G
            for i = 1:size(p, 2)
                G_i = [self.Jacobian_b(1:3,:,i).'; -self.Jacobian_b(1:3,:,i).'];
                G = blkdiag(G, G_i);
            end

            % d
            for i = 1:size(p, 2)
                d_i = [tau_max, tau_max, tau_max, tau_min, tau_min, tau_min].';
                d = vertcat(d, d_i);
            end

            % W
            for i = 1:size(ej, 2)
                W_i = [v(:,i).'; v(:,i).'];
                W = blkdiag(W, W_i);
            end

            % w
            for i = 1:size(ej, 2)
                w_i = [t_max, t_min].';
                w = vertcat(w, w_i);
            end

            % Combine constraints for linprog
            % The decision variables are [f; t; cxy]
            % Number of variables
            num_f = size(A1, 2);
            num_t = size(A3, 2);
            num_cxy = size(cxy, 1);  % Adjusted to match the size of cxy

            % Objective function: -ai.' * cxy (maximize ai.' * cxy -> minimize -ai.' * cxy)
            f_obj = [-ai; zeros(num_f + num_t, 1)];

            % Constraints: A1*f + A2*cxy + A3*t = u (Equality constraint)
            Aeq = [A1, A3, A2];
            beq = u;
    
            % Inequality constraints: B*f <= 0, G*f <= d, W*t <= w
            A_ineq = [B, zeros(size(B, 1), num_t + num_cxy);
                G, zeros(size(G, 1), num_t + num_cxy);
                zeros(size(W, 1), num_f), W, zeros(size(W, 1), num_cxy)];
            b_ineq = [zeros(size(B, 1), 1);
                d;
                w];

            % Bounds: lb <= x <= ub
            lb = [];
            ub = [];

            % Options for linprog
            options = optimoptions('linprog', 'Display', 'none');

            % Solve the LP
            [opt_vars, ~, exitflag] = linprog(f_obj, A_ineq, b_ineq, Aeq, beq, lb, ub, options);

            if exitflag == 1
                % Extract the optimal cxy and f
                cxy_opt = opt_vars(end-1:end);
                f_opt = opt_vars(1:num_f);
            else
                error('No feasible solution found');
            end
        end
    end
end
