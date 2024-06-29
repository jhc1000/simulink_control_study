classdef wlar_kinematics
    properties
        x_b2hr = 0.2576;  % b2hr
        z_b2hr = 0.0;    % b2hr
        z_hr2hp = 0.0636; % hr2hp
        z_hp2k = 0.300;   % hp2k
        z_k2w = 0.310;    % k2w
        y_b2hr = 0.065;   % b2hr
        y_hr2hp = 0.070;  % hr2hp
        y_hp2k = 0.029;   % hp2k
        y_k2w = 0.066;    % k2w
        % n1 = [1; 1; -1; -1];
        % n2 = [1; -1; 1; -1];
        % 
        % p_b_hp = zeros(4,3);
        % p_hp_k = zeros(4,3);
        % p_k_w = zeros(4,3);   
    end
    methods
        function self = init(~, self)
            self.x_b2hr = 0.2576;  % b2hr
            self.z_b2hr = 0.0;     % b2hr
            self.z_hr2hp = 0.0636; % hr2hp
            self.z_hp2k = 0.300;   % hp2k
            self.z_k2w = 0.310;    % k2w
            self.y_b2hr = 0.065;   % b2hr
            self.y_hr2hp = 0.070;  % hr2hp
            self.y_hp2k = 0.029;   % hp2k
            self.y_k2w = 0.066;    % k2w
            self.n1 = [1; 1; -1; -1];
            self.n2 = [1; -1; 1; -1];
    
            self.p.b_hr = zeros(3,4);
            self.p.hr_hp = zeros(3,4);
            self.p.hp_k = zeros(3,4);
            self.p.k_w = zeros(3,4);

            self.R.g_b = eye(3,3);
            self.T.g_b = eye(4,4);

            self.R.b_hr = zeros(3,3,4);
            self.R.hr_hp = zeros(3,3,4);
            self.R.hp_k = zeros(3,3,4);
            self.R.k_w = zeros(3,3,4);

        end
        
    end
    methods(Static)
        function self = forward_kinematics(self, q_base, p_base)
            import math_tools.*

            self.R.s_b = math_tools.rpyToRot(q_base(1), q_base(2), q_base(3));
            self.T.s_b = math_tools.homogeneous_matrix(self.R.g_b, p_base);
       
            for i = 1:4  
                % Base Coordinate  
                self.R.b_hr(:,:,i) = rpyToRot(self.q.hr(i), 0.0, 0.0);
                self.R.hr_hp(:,:,i) = rpyToRot(0.0, self.q.hp(i), 0.0);
                self.R.hp_k(:,:,i) = rpyToRot(0.0, self.q.k(i), 0.0);
                self.R.k_w(:,:,i) = rpyToRot(0.0, 0.0, 0.0);

                self.R.b_hp(:,:,i) = self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i);
                self.R.b_k(:,:,i) = self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i)*self.R.hp_k(:,:,i);
                self.R.b_w(:,:,i) = self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i)*self.R.hp_k(:,:,i)*self.R.k_w(:,:,i);

                self.p.b_hr(:,i) = [self.n1(i)*(self.x_b2hr); self.n2(i)*(self.y_b2hr); -(self.z_b2hr)];
                self.p.hr_hp(:,i) = [0.0; self.n2(i)*(self.y_hr2hp); -(self.z_hr2hp)];
                self.p.hp_k(:,i) = [0.0; self.n2(i)*(self.y_hp2k); -(self.z_hp2k)];
                self.p.k_w(:,i) = [0.0; -self.n2(i)*(self.y_k2w); -(self.z_hp2k)];

                self.T.b_hr(:,:,i) = math_tools.homogeneous_matrix(self.R.b_hr(:,:,i),self.p.b_hr(:,i));
                self.T.hr_hp(:,:,i) = math_tools.homogeneous_matrix(self.R.hr_hp(:,:,i),self.p.hr_hp(:,i));
                self.T.hp_k(:,:,i) = math_tools.homogeneous_matrix(self.R.hp_k(:,:,i),self.p.hp_k(:,i));
                self.T.k_w(:,:,i) = math_tools.homogeneous_matrix(self.R.k_w(:,:,i),self.p.k_w(:,i));

                self.T.b_w(:,:,i) = self.T.b_hr(:,:,i)*self.T.hr_hp(:,:,i)*self.T.hp_k(:,:,i)*self.T.k_w(:,:,i);
                self.p.b_w(:,i) = self.T.b_w(:,4,i);
                
                self.T.b_k(:,:,i) = self.T.b_hr(:,:,i)*self.T.hr_hp(:,:,i)*self.T.hp_k(:,:,i);
                self.p.b_k(:,i) = self.T.b_k(:,4,i);

                self.T.b_hp(:,:,i) = self.T.b_hr(:,:,i)*self.T.hr_hp(:,:,i);
                self.p.b_hp(:,i) = self.T.b_hp(:,4,i);
        
                % Space Coordinate
                self.R.s_hr(:,:,i) = self.R.s_b*self.R.b_hr(:,:,i);
                self.R.s_hp(:,:,i) = self.R.s_b*self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i);
                self.R.s_k(:,:,i) = self.R.s_b*self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i)*self.R.hp_k(:,:,i);
                self.R.s_w(:,:,i) = self.R.s_b*self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i)*self.R.hp_k(:,:,i)*self.R.k_w(:,:,i);

                self.T.s_hr(:,:,i) = self.T.s_b*self.T.b_hr(:,:,i);
                self.p.s_hr(:,i) = self.T.s_hr(:,4,i);
                
                self.T.s_hp(:,:,i) = self.T.s_b*self.T.b_hp(:,:,i);
                self.p.s_hp(:,i) = self.T.s_hp(:,4,i);
                
                self.T.s_k(:,:,i) = self.T.s_b*self.T.b_k(:,:,i);
                self.p.s_k(:,i) = self.T.s_k(:,4,i);
                
                self.T.s_w(:,:,i) = self.T.s_b*self.T.b_w(:,:,i);
                self.p.s_w(:,i) = self.T.s_w(:,4,i);
            end
        end
        function self = Jacobians(self, dot_q_base, dot_p_base)
            import math_tools.*
            
            self.S.s_b = [dot_p_base ; dot_q_base];
            self.Ad.s_b = math_tools.adjoint_matrix(self.R.s_b, self.p_base);

            for i = 1:4
                % Base Coordinate
                self.S.b_hr(:,i) = [self.n2(i)*self.y_b2hr;-self.n1(i)*self.x_b2hr;0;0;0;1];
                self.S.b_hp(:,i) = [(self.z_b2hr+self.z_hr2hp);0;self.n1(i)*self.x_b2hr;0;1;0];
                self.S.b_k(:,i) = [(self.z_b2hr+self.z_hr2hp+self.z_hp2k);0;self.n1(i)*self.x_b2hr;0;1;0];
                
                self.Ad.b_hr(:,:,i) = math_tools.adjoint_matrix(self.R.b_hr(:,:,i), self.p.b_hr(:,i));
                self.Ad.b_hp(:,:,i) = math_tools.adjoint_matrix(self.R.b_hp(:,:,i), self.p.b_hp(:,i));

                self.J.b_hr(:,:,i) = self.S.b_hr(:,i);
                self.J.b_hp(:,:,i) = self.Ad.b_hr(:,:,i)*self.S.b_hp(:,i);
                self.J.b_k(:,:,i) = self.Ad.b_hp(:,:,i)*self.S.b_k(:,i);

                % Space Coordinate
                self.S.s_hr(:,i) = self.Ad.s_b*self.S.b_hr(:,i);
                self.S.s_hp(:,i) = self.Ad.s_b*self.S.b_hp(:,i);
                self.S.s_k(:,i) = self.Ad.s_b*self.S.b_k(:,i);

                self.Ad.s_hr(:,:,i) = math_tools.adjoint_matrix(self.R.s_hr(:,:,i), self.p.s_hr(:,i));
                self.Ad.s_hp(:,:,i) = math_tools.adjoint_matrix(self.R.s_hp(:,:,i), self.p.s_hp(:,i));

                self.J.s_hr(:,:,i) = self.S.s_hr(:,i);
                self.J.s_hp(:,:,i) = self.Ad.s_hr(:,:,i)*self.S.s_hp(:,i);
                self.J.s_k(:,:,i) = self.Ad.s_hp(:,:,i)*self.S.s_k(:,i);


                self.Jacobian_b(:,:,i) = [self.J.b_hr(:,:,i) self.J.b_hp(:,:,i) self.J.b_k(:,:,i)];
                self.Jacobian_s(:,:,i) = [self.J.s_hr(:,:,i) self.J.s_hp(:,:,i) self.J.s_k(:,:,i)];
            end
        end
    end
end

