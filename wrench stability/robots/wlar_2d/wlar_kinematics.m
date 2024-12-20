classdef wlar_kinematics
    properties
        x_b2hr = 0.25825;  % b2hr
        z_b2hr = 0.0605;    % b2hr
        z_hr2hp = 0.094; % hr2hp
        z_hp2k = 0.3011;   % hp2k
        z_k2w = 0.3189;    % k2w
        y_b2hr = 0.283;   % b2hr
        y_hr2hp = 0.058;  % hr2hp
        y_hp2k = 0.0;   % hp2k
        y_k2w = 0.053;    % k2w
        % n1 = [1; 1; -1; -1];
        % n2 = [1; -1; 1; -1];
        % 
        % p_b_hp = zeros(4,3);
        % p_hp_k = zeros(4,3);
        % p_k_w = zeros(4,3);   
    end
    methods
        function self = init(~, self)
            self.x_b2hr = 0.25825;  % b2hr
            self.z_b2hr = 0.0605;     % b2hr
            self.z_hr2hp = 0.094; % hr2hp
            self.z_hp2k = 0.3011;   % hp2k
            self.z_k2w = 0.3539;    % k2w
            self.y_b2hr = 0.283;   % b2hr
            self.y_hr2hp = 0.058;  % hr2hp
            self.y_hp2k = -0.0;   % hp2k
            self.y_k2w = 0.0458;    % k2w 
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

            self.ropeejector.position.b(:,:,1) = [0.389345; 0.159; 0.0];
            self.ropeejector.position.b(:,:,2) = [0.389345; -0.159; 0.0];

            self.r_sheeve = 0.0735;

        end
        
    end
    methods(Static)
        function self = forward_kinematics(self, q_base, p_base)
            import math_tools.*

            self.R.s_b = math_tools.rpyToRot(q_base(1), q_base(2), q_base(3));
            self.T.s_b = math_tools.homogeneous_matrix(self.R.g_b, p_base);
       
            for i = 1:4  
                % Base Coordinate  
                self.R.b_hr(:,:,i) = rpyToRot(0.0, 0.0, self.q.hr(i));
                self.R.hr_hp(:,:,i) = rpyToRot(0.0, self.q.hp(i), 0.0);
                self.R.hp_k(:,:,i) = rpyToRot(0.0, self.q.k(i), 0.0);
                self.R.k_w(:,:,i) = rpyToRot(0.0, 0.0, 0.0);

                self.R.b_hp(:,:,i) = self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i);
                self.R.b_k(:,:,i) = self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i)*self.R.hp_k(:,:,i);
                self.R.b_w(:,:,i) = self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i)*self.R.hp_k(:,:,i)*self.R.k_w(:,:,i);

                self.p.b_hr(:,i) = [self.n1(i)*(self.x_b2hr); self.n2(i)*(self.y_b2hr); (self.z_b2hr)];
                self.p.hr_hp(:,i) = [0.0; self.n2(i)*(self.y_hr2hp); -(self.z_hr2hp)];
                self.p.hp_k(:,i) = [0.0; self.n2(i)*(self.y_hp2k); -(self.z_hp2k)];
                self.p.k_w(:,i) = [0.0; self.n2(i)*(-self.y_k2w); -(self.z_k2w)];

                self.T.b_hr(:,:,i) = math_tools.homogeneous_matrix(self.R.b_hr(:,:,i),self.p.b_hr(:,i));
                self.T.hr_hp(:,:,i) = math_tools.homogeneous_matrix(self.R.hr_hp(:,:,i),self.p.hr_hp(:,i));
                self.T.hp_k(:,:,i) = math_tools.homogeneous_matrix(self.R.hp_k(:,:,i),self.p.hp_k(:,i));
                self.T.k_w(:,:,i) = math_tools.homogeneous_matrix(self.R.k_w(:,:,i),self.p.k_w(:,i));

                self.T.b_w(:,:,i) = self.T.b_hr(:,:,i)*self.T.hr_hp(:,:,i)*self.T.hp_k(:,:,i)*self.T.k_w(:,:,i);
                self.p.b_w(:,i) = self.T.b_w(1:3,4,i);
                
                self.T.b_k(:,:,i) = self.T.b_hr(:,:,i)*self.T.hr_hp(:,:,i)*self.T.hp_k(:,:,i);
                self.p.b_k(:,i) = self.T.b_k(1:3,4,i);

                self.T.b_hp(:,:,i) = self.T.b_hr(:,:,i)*self.T.hr_hp(:,:,i);
                self.p.b_hp(:,i) = self.T.b_hp(1:3,4,i);
        
                % Space Coordinate
                self.R.s_hr(:,:,i) = self.R.s_b*self.R.b_hr(:,:,i);
                self.R.s_hp(:,:,i) = self.R.s_b*self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i);
                self.R.s_k(:,:,i) = self.R.s_b*self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i)*self.R.hp_k(:,:,i);
                self.R.s_w(:,:,i) = self.R.s_b*self.R.b_hr(:,:,i)*self.R.hr_hp(:,:,i)*self.R.hp_k(:,:,i)*self.R.k_w(:,:,i);

                self.T.s_hr(:,:,i) = self.T.s_b*self.T.b_hr(:,:,i);
                self.p.s_hr(:,i) = self.T.s_hr(1:3,4,i);
                
                self.T.s_hp(:,:,i) = self.T.s_b*self.T.b_hp(:,:,i);
                self.p.s_hp(:,i) = self.T.s_hp(1:3,4,i);
                
                self.T.s_k(:,:,i) = self.T.s_b*self.T.b_k(:,:,i);
                self.p.s_k(:,i) = self.T.s_k(1:3,4,i);
                
                self.T.s_w(:,:,i) = self.T.s_b*self.T.b_w(:,:,i);
                self.p.s_w(:,i) = self.T.s_w(1:3,4,i);
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
        function self = ascender_forward_kinematics(self, q_base, p_base)
            import math_tools.*

            self.R.s_b = math_tools.rpyToRot(q_base(1), q_base(2), q_base(3));
            self.T.s_b = math_tools.homogeneous_matrix(self.R.g_b, p_base);
       
            for i = 1:2
                % Base Coordinate  
                self.R.b_ej(:,:,i) = rpyToRot(0.0, 0.0, 0.0);

                self.p.b_ej(:,i) = self.ropeejector.position.b(:,:,i).';

                self.T.b_ej(:,:,i) = math_tools.homogeneous_matrix(self.R.b_ej(:,:,i),self.p.b_ej(:,i));
        
                % Space Coordinate
                self.R.s_ej(:,:,i) = self.R.s_b*self.R.b_ej(:,:,i);

                self.T.s_ej(:,:,i) = self.T.s_b*self.T.b_ej(:,:,i);
                self.p.s_ej(:,i) = self.T.s_ej(1:3,4,i);
                   

                self.v.s_ej_anc(:,i) = self.anchor.position(:,:,i) - self.p.s_ej(:,i);
                self.v.b_ej_anc(:,i) = self.R.s_b.'*self.v.s_ej_anc(:,i);

            end
        end
        function self = ascender_Jacobians(self, dot_q_base, dot_p_base)
            import math_tools.*

            self.S.s_b = [dot_p_base ; dot_q_base];
            self.Ad.s_b = math_tools.adjoint_matrix(self.R.s_b, self.p_base);
       
            for i = 1:2             
                self.J.b_asc_q(:,:,i) = [self.r_sheeve.*self.v.s_ej_anc(:,i)/norm(self.v.s_ej_anc(:,i)); zeros(3,1)];
                % self.J.s_asc_q(:,:,i) = [self.r_sheeve.*self.v.s_ej_anc(:,i)/norm(self.v.s_ej_anc(:,i)); zeros(3,1)];
                
            end
            self.asc_Jacobian_b(:,:) = [self.J.b_asc_q(:,:,1) self.J.b_asc_q(:,:,2)];
            % self.asc_Jacobian_s(:,:) = [self.J.s_asc_q(:,:,1) self.J.s_asc_q(:,:,2)];
        end
        function positions = get_local_positions(self,world_pos,dx,dy,dz,roll,pitch,yaw) 
            import math_tools.*
            leg_positions = vstack(world_pos, ones(1,4)); 
            T_m_mat = T_m(dx,dy,dz,roll,pitch,yaw);
            T_m_mat_inv = homog_transform_inverse(T_m_mat);

            % base to hr Pos Ver.2
            b_pos_FL = [self.x_b2hr, self.y_b2hr, self.z_b2hr, 1]';
            b_pos_FR = [self.x_b2hr, -self.y_b2hr, self.z_b2hr, 1]';
            b_pos_RL = [-self.x_b2hr, self.y_b2hr, self.z_b2hr, 1]';
            b_pos_RR = [-self.x_b2hr, -self.y_b2hr, self.z_b2hr, 1]';

            % base to end-effector
            b_end_FL = T_m_mat_inv*leg_positions(:,1);
            b_end_FR = T_m_mat_inv*leg_positions(:,2);
            b_end_RL = T_m_mat_inv*leg_positions(:,3);
            b_end_RR = T_m_mat_inv*leg_positions(:,4);

            % hr to end-effector
            hr_pos_end_FL = b_end_FL - b_pos_FL;
            hr_pos_end_FR = b_end_FR - b_pos_FR;
            hr_pos_end_RL = b_end_RL - b_pos_RL;
            hr_pos_end_RR = b_end_RR - b_pos_RR;

            positions = [hr_pos_end_FL(1:3),hr_pos_end_FR(1:3),hr_pos_end_RL(1:3),hr_pos_end_RR(1:3)];


        end
        function self = wheelleg_ik(self,leg_positions,dx,dy,dz,roll,pitch,yaw)
            import math_tools.*
            positions = self.kinematics.get_local_positions(self,leg_positions,dx,dy,dz,roll,pitch,yaw);
            disp("leg position : "+ string(leg_positions));
            disp("local position :"+string(positions));

            sym_1 = [1, 1, -1, -1];
            sym_2 = [1, -1, 1, -1];
            sym_3 = [-1, 1, 1, -1];
            sym_4 = [1, -1, -1, 1];

            hr_beta = zeros(4,1);
            hr_alpha = zeros(4,1);
            q_hr = zeros(4,1);
    
            rot_qhr = zeros(3, 3, 4);
            k = zeros(4,1);
            q_k = zeros(4,1);
            hp_phi = zeros(4,1);
            hp_psi = zeros(4,1);
            q_hp = zeros(4,1);

            related_ee_pos = zeros(3,4);

            for i =1:4
                x = positions(1,i);
                y = positions(2,i);
                z = positions(3,i);

                kl = (self.y_hr2hp + self.y_hp2k - self.y_k2w) / sqrt(x^2 + y^2);
                hr_beta(i) = acos(kl);

                if i == 1
                    hr_alpha(i) = atan2((x), (y));  % -pi ~ pi y is under
                    q_hr(i) = (hr_beta(i) - hr_alpha(i));

                elseif i == 2
                    hr_alpha(i) = atan2((x), (-y));  % -pi ~ pi y is under
                    q_hr(i) = -(hr_beta(i) - hr_alpha(i));

                elseif i == 3
                    hr_alpha(i) = atan2((-x), (y));  % -pi ~ pi y is under
                    q_hr(i) = -(hr_beta(i) - hr_alpha(i));

                elseif i == 4
                    hr_alpha(i) = atan2((-x), (-y)); % -pi ~ pi y is under
                    q_hr(i) = (hr_beta(i) - hr_alpha(i));
                end
                
                if i == 1
                    if q_hr(i) > pi/2
                        q_hr(i) = q_hr(i) - pi;
                    elseif q_hr(i) < -pi/2
                        q_hr(i) = q_hr(i) + pi;
                    end
                elseif i == 2
                    if q_hr(i) > pi/2
                        q_hr(i) = q_hr(i) - pi;
                    elseif q_hr(i) < -pi/2
                        q_hr(i) = q_hr(i) + pi;
                    end
                elseif i == 3
                    if q_hr(i) > pi/2
                        q_hr(i) =q_hr(i) - pi;
                    elseif q_hr(i) < -pi/2
                        q_hr(i) = q_hr(i)+ pi;
                    end
                elseif i == 4
                    if q_hr(i) > pi/2
                        q_hr(i) = q_hr(i) - pi;
                    elseif q_hr(i) < -pi/2
                        q_hr(i) = q_hr(i) + pi;
                    end
                end

                rot_qhr(:, :, i) = [cos(q_hr(i)), -sin(q_hr(i)), 0.0;
                                    sin(q_hr(i)), cos(q_hr(i)), 0.0;
                                    0.0, 0.0, 1.0];

                related_ee_pos(:,i) = rot_qhr(:, :, i)'* positions(:,i);


                x1 = related_ee_pos(1,i);
                y1 = related_ee_pos(2,i);
                z1 = related_ee_pos(3,i);
                
                k(i) = sqrt(x1^2 + (z1 + self.z_hr2hp)^2);
            
                q_k(i) = -sym_4(i) * (acos((k(i)^2 - self.z_hp2k^2 - self.z_k2w^2) / (2 * self.z_hp2k * self.z_k2w)));
                
                hp_phi(i) = sym_2(i) * ((atan2(x1, (z1 + self.z_hr2hp))) - pi);
                hp_psi(i) = atan2(self.z_k2w * sin(q_k(i)), self.z_hp2k + self.z_k2w * cos(q_k(i)));

                q_hp(i) = (hp_phi(i) + hp_psi(i));
    
                if q_hp(i) > pi
                    q_hp(i) = q_hp(i) - pi*2;
                elseif q_hp(i) < -pi
                    q_hp(i) = q_hp(i) + pi*2;
                end

            end

            angles = [q_hr, sym_2'.*q_hp, -sym_2'.*q_k];
            angles_degree = rad2deg(angles);
            
            self.ik_angle = angles;
            self.ik_angle_degree = angles_degree;
        end
        function self = wheelleg_ik_numeric(self,target_leg_positions,dx,dy,dz,roll,pitch,yaw)
            import math_tools.*
            % 수치해석적 역기구학 계산 (Newton-Raphson 방법)
            target_leg_positions = self.p.b_w;
            disp("target_leg_positions:");
            disp(target_leg_positions);

            self.tol = 1e-3;  % 수렴 허용 오차
            self.max_iter = 10000;  % 최대 반복 횟수
            self.step_size = 0.0001;  % 스텝 사이즈

            q.hr_initial = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
            q.hp_initial = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
            q.k_initial = [deg2rad(80); deg2rad(80); deg2rad(-80); deg2rad(-80)];

            self.q.hr = q.hr_initial;
            self.q.hp = q.hp_initial;
            self.q.k = q.k_initial;

            for iter = 1:self.max_iter
                % Forward Kinematics 계산
                self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
                disp("self.p.b_w:");
                disp(self.p.b_w);
                % 현재 발의 위치와 목표 위치의 오차 계산
                error = self.p.b_w - target_leg_positions;
                error_norm = norm(error);
                disp("error:");
                disp(error)
                % 수렴 조건 확인
                if error_norm < self.tol
                    fprintf('수렴 완료: %d번째 반복에서 오차 = %.6f\n', iter, error_norm);
                    break;
                end


                % Jacobians 계산
                self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);

                % 각 다리마다 Jacobian을 사용하여 오차에 대한 관절 각도 변화 계산
                delta_theta = zeros(3, 4);  % 각도 변화 초기화
                for i = 1:4
                    J = self.Jacobian_b(1:3,:,i);  % 3x3 Jacobian (각 다리의 위치에 대한 것만 사용)
                    delta_theta(:,i) = pinv(J) * error(:,i);  % Jacobian의 유사 역행렬로 각도 변화 계산
                end

                % 관절 각도 업데이트
                self.q.hr = self.q.hr - self.step_size * delta_theta(1,:)';
                self.q.hp = self.q.hp - self.step_size * delta_theta(2,:)';
                self.q.k = self.q.k - self.step_size * delta_theta(3,:)';
                
                % 오차 출력
                fprintf('반복 %d: 오차 = %.6f\n', iter, error_norm);
            end
            
            if iter == self.max_iter
                fprintf('최대 반복 횟수 도달: 오차 = %.6f\n', error_norm);
            end
        end
    end
end

