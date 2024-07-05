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
                % self.psuedo_inverse_jacobian(:,:,i) = (self.Jacobian_b(:,:,i)*self.Jacobian_b(:,:,i).')\self.Jacobian_b(:,:,i);
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
                else
                    self.leg_actuation_wrench_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d,[self.leg_actuation_wrench_polytope(:,1,i) self.leg_actuation_wrench_polytope(:,5,i) self.leg_actuation_wrench_polytope(:,3,i)]);
                end
            end
            self.leg_actuation_wrench_polytope_total_convhull = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
        end
        function self = compute_contact_wrench_polytope(self)
            self.leg_friction_space(:,:,1) = 10000.*[self.mu self.mu 1.0;
                -self.mu self.mu 1.0;
                self.mu -self.mu 1.0;
                -self.mu -self.mu 1.0;
                0.0 0.0 0.0];

            self.leg_friction_space(:,:,2) = 10000.*[self.mu self.mu 1.0;
                -self.mu self.mu 1.0;
                self.mu -self.mu 1.0;
                -self.mu -self.mu 1.0;
                0.0 0.0 0.0];

            self.leg_friction_space(:,:,3) = 10000.*[self.mu self.mu 1.0;
                -self.mu self.mu 1.0;
                self.mu -self.mu 1.0;
                -self.mu -self.mu 1.0;
                0.0 0.0 0.0];

            self.leg_friction_space(:,:,4) = 10000.*[self.mu self.mu 1.0;
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
                else
                    self.leg_contact_wrench_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_contact_wrench_polytope_total_3d,[self.leg_contact_wrench_polytope(:,1,i) self.leg_contact_wrench_polytope(:,5,i) self.leg_contact_wrench_polytope(:,3,i)]);
                end
            end
            self.leg_contact_wrench_polytope_total_convhull = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
        end
        function self = compute_ascender_wrench_polytope(self)
            self.asc_tension_space = [self.ASC_L_tension_lim(1) self.ASC_R_tension_lim(1);
                self.ASC_L_tension_lim(2) self.ASC_R_tension_lim(1);
                self.ASC_L_tension_lim(1) self.ASC_R_tension_lim(2);
                self.ASC_L_tension_lim(2) self.ASC_R_tension_lim(2);];

            self.asc_wrench_matrix = [self.v.b_ej_anc(:,1) self.v.b_ej_anc(:,2);
                cross(self.p.b_ej(:,1),self.v.b_ej_anc(:,1)) cross(self.p.b_ej(:,2),self.v.b_ej_anc(:,2))];
            for i = 1:4
                self.ascender_wrench_polytope(i,:) = (self.asc_wrench_matrix*self.asc_tension_space(i,:).').';
                self.ascender_force_polytope(i,:) = [self.ascender_wrench_polytope(i,1) self.ascender_wrench_polytope(i,2) self.ascender_wrench_polytope(i,3)];
                self.ascender_moment_polytope(i,:) = [self.ascender_wrench_polytope(i,4) self.ascender_wrench_polytope(i,5) self.ascender_wrench_polytope(i,6)];
            end
            self.asc_wrench_polytope_3d = [self.ascender_wrench_polytope(:,1) self.ascender_wrench_polytope(:,5) self.ascender_wrench_polytope(:,3)];
            self.asc_wrench_polytope_convhull = Polyhedron(self.asc_wrench_polytope_3d);
        end
    end
end

