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
        function self = compute_force_wrench_polytope(self)
            self.leg_torque_space(:,:,1) = [self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
                -self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
                self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
                self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
                -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
                self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
                -self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
                -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(3)];

            self.leg_torque_space(:,:,2) = [self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
                -self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
                self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
                self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
                -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
                self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
                -self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
                -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(3)];

            self.leg_torque_space(:,:,3) = [self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
                -self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
                self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
                self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
                -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
                self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
                -self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
                -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(3)];

            self.leg_torque_space(:,:,4) = [self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
                -self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
                self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
                self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
                -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
                self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
                -self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
                -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(3)];

            for i=1:4
                self.psuedo_inverse_jacobian(:,:,i) = pinv(self.Jacobian_b(:,:,i));
                for j=1:8
                    self.leg_actuation_wrench_polytope(j,:,i) = ((self.psuedo_inverse_jacobian(:,:,i).'*self.leg_torque_space(j,:,i).').');
                    self.leg_actuation_force_polytope(j,:,i) = [self.leg_actuation_wrench_polytope(j,1,i) self.leg_actuation_wrench_polytope(j,2,i) self.leg_actuation_wrench_polytope(j,3,i)];
                    self.leg_actuation_moment_polytope(j,:,i) = cross(self.p.b_w(:,i).',self.leg_actuation_force_polytope(j,:,i));
                end
            end

            for i=1:4
                if i == 1
                    self.leg_actuation_wrench_polytope_total_3d = [self.leg_actuation_wrench_polytope(:,1,i) self.leg_actuation_wrench_polytope(:,5,i) self.leg_actuation_wrench_polytope(:,3,i)];
                    self.leg_actuation_wrench_polytope_total = [self.leg_actuation_wrench_polytope(:,1,i) self.leg_actuation_wrench_polytope(:,2,i) self.leg_actuation_wrench_polytope(:,3,i)];
                else
                    self.leg_actuation_wrench_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d,[self.leg_actuation_wrench_polytope(:,1,i) self.leg_actuation_wrench_polytope(:,5,i) self.leg_actuation_wrench_polytope(:,3,i)]);
                    self.leg_actuation_wrench_polytope_total = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d,[self.leg_actuation_wrench_polytope(:,1,i) self.leg_actuation_wrench_polytope(:,2,i) self.leg_actuation_wrench_polytope(:,3,i)]);
                end
            end
            % self.leg_actuation_wrench_polytope_total_convhull = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
            self.leg_actuation_wrench_polytope_total_convhull = Polyhedron(self.leg_actuation_wrench_polytope_total);
        end
        function self = compute_contact_wrench_polytope(self)
            self.leg_friction_space(:,:,1) = 5000.*[self.mu self.mu 1.0;
                -self.mu self.mu 1.0;
                self.mu -self.mu 1.0;
                -self.mu -self.mu 1.0;
                0.0 0.0 0.0];

            self.leg_friction_space(:,:,2) = 5000.*[self.mu self.mu 1.0;
                -self.mu self.mu 1.0;
                self.mu -self.mu 1.0;
                -self.mu -self.mu 1.0;
                0.0 0.0 0.0];

            self.leg_friction_space(:,:,3) = 5000.*[self.mu self.mu 1.0;
                -self.mu self.mu 1.0;
                self.mu -self.mu 1.0;
                -self.mu -self.mu 1.0;
                0.0 0.0 0.0];

            self.leg_friction_space(:,:,4) = 5000.*[self.mu self.mu 1.0;
                -self.mu self.mu 1.0;
                self.mu -self.mu 1.0;
                -self.mu -self.mu 1.0;
                0.0 0.0 0.0];

            for i=1:4
                for j=1:5
                    self.leg_contact_force_polytope(j,:,i) = self.leg_friction_space(j,:,i).';
                    self.leg_contact_moment_polytope(j,:,i) = cross(self.p.b_w(:,i).',self.leg_contact_force_polytope(j,:,i).');
                    self.leg_contact_wrench_polytope(j,:,i) = horzcat(self.leg_contact_force_polytope(j,:,i),self.leg_contact_moment_polytope(j,:,i));
                end
            end
            for i=1:4
                if i == 1
                    self.leg_contact_wrench_polytope_total_3d = [self.leg_contact_wrench_polytope(:,1,i) self.leg_contact_wrench_polytope(:,5,i) self.leg_contact_wrench_polytope(:,3,i)];
                    self.leg_contact_wrench_polytope_total = [self.leg_contact_wrench_polytope(:,1,i) self.leg_contact_wrench_polytope(:,2,i) self.leg_contact_wrench_polytope(:,3,i)];
                else
                    self.leg_contact_wrench_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_contact_wrench_polytope_total_3d,[self.leg_contact_wrench_polytope(:,1,i) self.leg_contact_wrench_polytope(:,5,i) self.leg_contact_wrench_polytope(:,3,i)]);
                    self.leg_contact_wrench_polytope_total = geometry_computation.minkowskiSum(self.leg_contact_wrench_polytope_total,[self.leg_contact_wrench_polytope(:,1,i) self.leg_contact_wrench_polytope(:,2,i) self.leg_contact_wrench_polytope(:,3,i)]);
                end
            end
            % self.leg_contact_wrench_polytope_total_convhull = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
            self.leg_contact_wrench_polytope_total_convhull = Polyhedron(self.leg_contact_wrench_polytope_total);
        end
        function self = compute_ascender_wrench_polytope(self)
            self.asc_tension_space = [self.ASC_L_tension_lim(1) self.ASC_R_tension_lim(1);
                self.ASC_L_tension_lim(2) self.ASC_R_tension_lim(1);
                self.ASC_L_tension_lim(1) self.ASC_R_tension_lim(2);
                self.ASC_L_tension_lim(2) self.ASC_R_tension_lim(2);];

            self.asc_torque_space = [self.ASC_L_tau_lim self.ASC_R_tau_lim;
                -self.ASC_L_tau_lim self.ASC_R_tau_lim;
                self.ASC_L_tau_lim -self.ASC_R_tau_lim;
                -self.ASC_L_tau_lim -self.ASC_R_tau_lim];

            self.asc_wrench_matrix = [self.v.b_ej_anc(:,1) self.v.b_ej_anc(:,2);
                cross(self.p.b_ej(:,1),self.v.b_ej_anc(:,1)) cross(self.p.b_ej(:,2),self.v.b_ej_anc(:,2))];

            self.asc_psuedo_inverse_jacobian(:,:) = pinv(self.asc_Jacobian_b(:,:));
            for i = 1:4
                self.ascender_wrench_polytope(i,:) = (self.asc_wrench_matrix*self.asc_tension_space(i,:).').';
                self.ascender_force_polytope(i,:) = [self.ascender_wrench_polytope(i,1) self.ascender_wrench_polytope(i,2) self.ascender_wrench_polytope(i,3)];
                self.ascender_moment_polytope(i,:) = [self.ascender_wrench_polytope(i,4) self.ascender_wrench_polytope(i,5) self.ascender_wrench_polytope(i,6)];
            
                self.ascender_actuation_force_polytope(i,:) = ((self.asc_psuedo_inverse_jacobian(:,1:3).'*self.asc_torque_space(i,:).').');
            end
            self.asc_wrench_polytope_3d = [self.ascender_wrench_polytope(:,1) self.ascender_wrench_polytope(:,5) self.ascender_wrench_polytope(:,3)];
            % self.asc_wrench_polytope_convhull = Polyhedron(self.asc_wrench_polytope_3d);
            self.asc_wrench_polytope_convhull = Polyhedron(self.ascender_force_polytope);
            self.ascender_actuation_force_polytope_convhull = Polyhedron(self.ascender_actuation_force_polytope);
        end
        function Yfa = feasibleRegionIP(self, cxy, cz, alpha, R_sb, p, n, mu, tau_min, tau_max, ej, v, t_min, t_max, epsilon)

            % Initialize outer and inner approximations of the feasible region
            Youter = geometry_computation.initialOuterApproximation(p);
            Yinner = geometry_computation.initialInnerApproximation(p);

            % Define the stopping criterion
            while geometry_computation.area(Youter) - geometry_computation.area(Yinner) > epsilon
                % I) Compute the edges of Yinner
                edges = geometry_computation.computeEdges(Yinner);

                % II) Pick ai based on the edge cutting off the largest fraction of Youter
                ai = geometry_computation.pickEdge(edges, Youter);

                % III) Solve the LP
                [cxy_opt, f_opt] = geometry_computation.solveLP(self, ai, cxy, alpha, R_sb, p, n, mu, tau_min, tau_max, ej, v, t_min, t_max);

                % IV) Update the outer approximation Youter
                Youter = geometry_computation.updateOuterApproximation(Youter, cxy_opt);

                % V) Update the inner approximation Yinner
                Yinner = geometry_computation.updateInnerApproximation(Yinner, cxy_opt, f_opt);
            end

            % Return the final feasible region
            Yfa = Yinner;
        end

        function Youter = initialOuterApproximation(p)
            % Ensure p is in the correct format (3xN matrix)
            if size(p, 1) ~= 3
                p = p'; % Transpose if p is given as a row vector
            end

            % Check if p has enough points to form a convex hull
            if size(p, 2) < 3
                error('At least 3 points are needed to form a convex hull in 2D');
            end

            % Compute the convex hull of the points
            try
                k = convhull(p(1:2,:).');
            catch
                error('Failed to compute the convex hull. Check input points.');
            end

            % Initialize Youter as the vertices of the convex hull
            Youter = p(:, k);
        end



        function Yinner = initialInnerApproximation(p)
            % Compute the centroid of the polygon
            centroid = mean(p, 2);

            % Scaling factor to reduce the area to one-fifth
            scale_factor = sqrt(1/5);

            % Initialize Yinner as an empty array
            Yinner = zeros(size(p));

            % Scale each point towards the centroid
            for i = 1:size(p, 2)
                Yinner(:, i) = centroid + scale_factor * (p(:, i) - centroid);
            end
        end

        function edges = computeEdges(Yinner)
            % Compute the edges of the inner approximation Yinner
            % This function returns a list of edges of the current inner approximation
            % Each edge is represented as a pair of consecutive vertices

            % Number of vertices in Yinner
            num_points = size(Yinner, 2);

            % Initialize an empty array to hold the edges
            edges = zeros(num_points, 4);

            % Loop through each vertex and form edges with the next vertex
            for i = 1:num_points
                % Define the next index (wrap around to the first point)
                next_i = mod(i, num_points) + 1;

                % Form the edge from vertex i to vertex next_i
                edges(i, :) = [Yinner(1, i), Yinner(2, i), Yinner(1, next_i), Yinner(2, next_i)];
            end
        end

        function ai = pickEdge(edges, Youter)
            % Pick the edge that cuts off the largest fraction of the outer approximation
            % Implement your heuristic to pick the most effective edge

            num_edges = size(edges, 1);
            min_area = inf;
            best_edge = [];

            % Loop through each edge to determine which one cuts off the largest fraction
            for i = 1:num_edges
                edge = edges(i, :);

                % Calculate the new area of Youter after cutting off the area under this edge
                % Create a polygon excluding the area under the current edge
                clipped_polygon = geometry_computation.clipPolygon(Youter, edge);

                % Calculate the area of the clipped polygon
                new_area = polyarea(clipped_polygon(1, :), clipped_polygon(2, :));

                % Check if this edge cuts off the largest fraction (smallest remaining area)
                if new_area < min_area
                    min_area = new_area;
                    best_edge = edge;
                end
            end

            ai = best_edge;
        end
        function clipped_polygon = clipPolygon(Youter, edge)
            % Clip the polygon Youter with the edge to remove the area under the edge
            % using the Sutherland-Hodgman algorithm

            % Extract the points of the edge
            x1 = edge(1);
            y1 = edge(2);
            x2 = edge(3);
            y2 = edge(4);

            % Define the clipping line
            clipper = [x1 y1; x2 y2];

            % Perform the clipping
            clipped_polygon = geometry_computation.sutherlandHodgman(Youter', clipper);

            % Transpose back to match the original format
            clipped_polygon = clipped_polygon';
        end

        function outputList = sutherlandHodgman(subjectPolygon, clipPolygon)
            % Sutherland-Hodgman algorithm for polygon clipping
            % subjectPolygon is a Nx2 matrix of the polygon vertices
            % clipPolygon is a 2x2 matrix of the clipping edge endpoints

            outputList = subjectPolygon;

            for i = 1:size(clipPolygon, 1)
                inputList = outputList;
                outputList = [];
                A = clipPolygon(i, :);
                B = clipPolygon(mod(i, size(clipPolygon, 1)) + 1, :);

                for j = 1:size(inputList, 1)
                    P = inputList(j, :);
                    Q = inputList(mod(j, size(inputList, 1)) + 1, :);

                    if geometry_computation.isInside(Q, A, B)
                        if ~geometry_computation.isInside(P, A, B)
                            outputList = [outputList; geometry_computation.intersection(P, Q, A, B)];
                        end
                        outputList = [outputList; Q];
                    elseif geometry_computation.isInside(P, A, B)
                        outputList = [outputList; geometry_computation.intersection(P, Q, A, B)];
                    end
                end
            end
        end

        function inside = isInside(P, A, B)
            % Check if point P is inside the clipping boundary defined by A and B
            inside = (B(1) - A(1)) * (P(2) - A(2)) > (B(2) - A(2)) * (P(1) - A(1));
        end

        function I = intersection(P, Q, A, B)
            % Compute the intersection point of line PQ with line AB
            a1 = Q(2) - P(2);
            b1 = P(1) - Q(1);
            c1 = a1 * P(1) + b1 * P(2);

            a2 = B(2) - A(2);
            b2 = A(1) - B(1);
            c2 = a2 * A(1) + b2 * A(2);

            det = a1 * b2 - a2 * b1;

            I = [(b2 * c1 - b1 * c2) / det, (a1 * c2 - a2 * c1) / det];
        end

        function [cxy_opt, f_opt] = solveLP(self, ai, cxy, alpha, R_sb, p, n, mu, tau_min, tau_max, ej, v, t_min, t_max)
            % Solve the linear programming problem
            % Implement the LP formulation based on your constraints and objective function
            % A1, A2, B, G, u, d should be defined according to the problem's constraints
            m = self.model.totalmass;
            g = [-9.8*sin(alpha), 0, -9.8*cos(alpha)].';
            P = [1 0 0; 0 1 0];
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
            for i=1:width(p)
                A_upperbar(:,:,i) = [eye(3); math_tools.skew(p(:,i))];
                A1 = horzcat(A1,A_upperbar(:,:,i));
            end

            % A2
            A2 = [zeros(3,2); -m.*cross(repmat(g, 1, 2), P.')];

            % A3
            for i=1:width(ej)
                A_underbar(:,:,i) = [eye(3); math_tools.skew(ej(:,i))];
                A3 = horzcat(A1,A_underbar(:,:,i));
            end

            % u
            u = [-m.*g; zeros(3)];

            % B
            b = [([1;0;0]-mu*[0;0;1]).';
                ([0;1;0]-mu*[0;0;1]).';
                -([1;0;0]+mu*[0;0;1]).';
                -([1;0;0]+mu*[0;0;1]).'];
            for i=1:width(p)
                B = blkdiag(B,b);
            end

            % G
            for i=1:width(p)
                G_i = [self.Jacobian_s(1:3,:,i).'; -self.Jacobian_s(1:3,:,i).'];
                G = blkdiag(G,G_i);
            end

            % d
            for i=1:width(p)
                d_i = [tau_max, tau_min].';
                d = vertcat(d, d_i);
            end

            % W
            for i=1:width(ej)
                W_i = [v(:,i).'; v(:,i).'];
                W = blkdiag(W,W_i);
            end

            % w
            for i=1:width(ej)
                w_i = [t_max, t_min].';
                w = vertcat(w, w_i);
            end

            % Combine constraints for linprog
            % The decision variables are [f; t; cxy]
            % Number of variables
            num_f = size(A1, 2);
            num_t = size(A3, 2);
            num_cxy = size(cxy, 1);

            % Objective function: -ai.' * cxy (maximize ai.' * cxy -> minimize -ai.' * cxy)
            f_obj = [-ai; zeros(num_f + num_t, 1)];

            % Constraints: Aeq * x = beq
            Aeq = [A1, A3, A2];
            beq = u;

            % Inequality constraints: A * x <= b
            A_ineq = [B, zeros(size(B,1), num_t + num_cxy);
                G, zeros(size(G,1), num_t + num_cxy);
                zeros(size(W,1), num_f), W, zeros(size(W,1), num_cxy)];
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

        function Youter = updateOuterApproximation(Youter, cxy_opt)
            % Update the outer approximation based on the new optimal point cxy_opt
            % Add the new point to the vertices of Youter and compute the convex hull

            % Add the new point to the list of vertices
            Youter = [Youter, cxy_opt'];

            % Compute the convex hull of the updated points
            K = convhull(Youter(1, :), Youter(2, :));

            % Update Youter to include only the points on the convex hull
            Youter = Youter(:, K);
        end

        function Yinner = updateInnerApproximation(Yinner, cxy_opt, f_opt)
            % Update the inner approximation based on the new optimal point cxy_opt and forces f_opt
            % Add the new point to the vertices of Yinner and compute the convex hull

            % Add the new point to the list of vertices
            Yinner = [Yinner, cxy_opt'];

            % Compute the convex hull of the updated points
            K = convhull(Yinner(1, :), Yinner(2, :));

            % Update Yinner to include only the points on the convex hull
            Yinner = Yinner(:, K);
        end

        function area_val = area(Y)
            % Compute the area of the region Y
            % Y is a 2xN matrix of the polygon vertices
            area_val = polyarea(Y(1, :), Y(2, :));
        end
    end
end

