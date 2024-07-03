classdef plotting_tools
    properties
    end

    methods(Static)
        function fig = plot_robot_base(self)
            fig = figure;
            h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
            xlim(pt,[-1 1]);ylim(pt,'manual');ylim(pt,[-1 1]);zlim(pt,'manual');
            zlim(pt,[-1 1]);axis equal;grid on;
            % view(90,90);
            % view(0,0);
            title('WLAR ForwardKinematics Simulation')
            hold on
            for i = 1:4
                point1.x{i} = self.p.b_hr(1,i); point1.y{i} = self.p.b_hr(2,i); point1.z{i} = self.p.b_hr(3,i);
                point2.x{i} = self.p.b_hp(1,i); point2.y{i} = self.p.b_hp(2,i); point2.z{i} = self.p.b_hp(3,i);
                point3.x{i} = self.p.b_k(1,i); point3.y{i} = self.p.b_k(2,i); point3.z{i} = self.p.b_k(3,i);
                point4.x{i} = self.p.b_w(1,i); point4.y{i} = self.p.b_w(2,i); point4.z{i} = self.p.b_w(3,i);
                L1{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','r','LineWidth',2);
                L2{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','b','LineWidth',2);
                L3{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','g','LineWidth',2);
                set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
                set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
                set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
            end
            Lb{1} = line([point1.x{1}, point1.x{2}],[point1.y{1},point1.y{2}],[point1.z{1},point1.z{2}],'Color','cyan','LineWidth',3);
            Lb{2} = line([point1.x{2}, point1.x{4}],[point1.y{2},point1.y{4}],[point1.z{2},point1.z{4}],'Color','b','LineWidth',3);
            Lb{3} = line([point1.x{3}, point1.x{4}],[point1.y{3},point1.y{4}],[point1.z{3},point1.z{4}],'Color','b','LineWidth',3);
            Lb{4} = line([point1.x{3}, point1.x{1}],[point1.y{3},point1.y{1}],[point1.z{3},point1.z{1}],'Color','b','LineWidth',3);
            Lw{1} = line([point4.x{1}, point4.x{2}],[point4.y{1},point4.y{2}],[point4.z{1},point4.z{2}],'Color','k','LineWidth',1);
            Lw{2} = line([point4.x{2}, point4.x{4}],[point4.y{2},point4.y{4}],[point4.z{2},point4.z{4}],'Color','k','LineWidth',1);
            Lw{3} = line([point4.x{3}, point4.x{4}],[point4.y{3},point4.y{4}],[point4.z{3},point4.z{4}],'Color','k','LineWidth',1);
            Lw{4} = line([point4.x{3}, point4.x{1}],[point4.y{3},point4.y{1}],[point4.z{3},point4.z{1}],'Color','k','LineWidth',1);

            xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
        end
        function fig = plot_robot_space(self)
            fig = figure;
            h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
            xlim(pt,[-1 1]);ylim(pt,'manual');ylim(pt,[-1 1]);zlim(pt,'manual');
            zlim(pt,[-1 1]);axis equal;grid on;
            % view(90,90);
            % view(0,0);
            title('WLAR ForwardKinematics Simulation')
            hold on
            for i = 1:4
                point1.x{i} = self.p.s_hr(1,i); point1.y{i} = self.p.s_hr(2,i); point1.z{i} = self.p.s_hr(3,i);
                point2.x{i} = self.p.s_hp(1,i); point2.y{i} = self.p.s_hp(2,i); point2.z{i} = self.p.s_hp(3,i);
                point3.x{i} = self.p.s_k(1,i); point3.y{i} = self.p.s_k(2,i); point3.z{i} = self.p.s_k(3,i);
                point4.x{i} = self.p.s_w(1,i); point4.y{i} = self.p.s_w(2,i); point4.z{i} = self.p.s_w(3,i);
                L1{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','r','LineWidth',2);
                L2{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','b','LineWidth',2);
                L3{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','g','LineWidth',2);
                set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
                set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
                set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
            end
            Lb{1} = line([point1.x{1}, point1.x{2}],[point1.y{1},point1.y{2}],[point1.z{1},point1.z{2}],'Color','cyan','LineWidth',3);
            Lb{2} = line([point1.x{2}, point1.x{4}],[point1.y{2},point1.y{4}],[point1.z{2},point1.z{4}],'Color','b','LineWidth',3);
            Lb{3} = line([point1.x{3}, point1.x{4}],[point1.y{3},point1.y{4}],[point1.z{3},point1.z{4}],'Color','b','LineWidth',3);
            Lb{4} = line([point1.x{3}, point1.x{1}],[point1.y{3},point1.y{1}],[point1.z{3},point1.z{1}],'Color','b','LineWidth',3);
            Lw{1} = line([point4.x{1}, point4.x{2}],[point4.y{1},point4.y{2}],[point4.z{1},point4.z{2}],'Color','k','LineWidth',1);
            Lw{2} = line([point4.x{2}, point4.x{4}],[point4.y{2},point4.y{4}],[point4.z{2},point4.z{4}],'Color','k','LineWidth',1);
            Lw{3} = line([point4.x{3}, point4.x{4}],[point4.y{3},point4.y{4}],[point4.z{3},point4.z{4}],'Color','k','LineWidth',1);
            Lw{4} = line([point4.x{3}, point4.x{1}],[point4.y{3},point4.y{1}],[point4.z{3},point4.z{1}],'Color','k','LineWidth',1);

            for i=1:2
                plot3(self.p.s_ej(1,i),self.p.s_ej(2,i),self.p.s_ej(3,i),'.','Color','cyan','MarkerSize',10)
                plot3(self.anchor.position(1,1,i),self.anchor.position(2,1,i),self.anchor.position(3,1,i),'.','Color','cyan','MarkerSize',10)
                L_rope{i} = line([self.p.s_ej(1,i), self.anchor.position(1,1,i)],[self.p.s_ej(2,i),self.anchor.position(2,1,i)],[self.p.s_ej(3,i),self.anchor.position(3,1,i)],'Color','cyan','LineWidth',3);
            end

            for i=1:4
                % [k1,av1] = convhull(0.000025.*self.leg_force_polytope(:,:,i));
                % trisurf(k1,0.00025.*self.leg_force_polytope(:,1,i)+self.p.s_w(1,i),0.00025.*self.leg_force_polytope(:,2,i)+self.p.s_w(2,i),0.00025.*self.leg_force_polytope(:,3,i)+self.p.s_w(3,i),'FaceColor','green')
                P1 = Polyhedron(self.p.s_w(:,i).'+0.00025.*self.leg_force_polytope(:,:,i));
                P1.plot('color', 'green', 'alpha', 0.5);
            end

            for i=1:4
                % [k2,av2] = convhull(0.000025.*self.leg_friction_polytope(:,:,i));
                % trisurf(k2,0.000025.*self.leg_friction_polytope(:,1,i)+self.p.s_w(1,i),0.000025.*self.leg_friction_polytope(:,2,i)+self.p.s_w(2,i),0.000025.*self.leg_friction_polytope(:,3,i)+self.p.s_w(3,i),'FaceColor','r')
                P2 = Polyhedron(self.p.s_w(:,i).'+0.000025.*self.leg_friction_polytope(:,:,i));
                P2.plot('color', 'red', 'alpha', 0.5);
            end
            hold off

            xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
        end
        function fig = plot_force_polytopes(self)
            self = geometry_computation.compute_force_wrench_polytope(self);

            fig = figure;
            tiledlayout(2,2);
            for i=1:4
                ax(i) = nexttile;
                P1 = Polyhedron(self.leg_force_polytope(:,:,i));
                % plot3(self.leg_force_wrench_polytope(:,1,i), self.leg_force_wrench_polytope(:,5,i), self.leg_force_wrench_polytope(:,3,i),".","Color","r","MarkerSize",10)
                % [k1,av1] = convhull([self.leg_force_wrench_polytope(:,1,i), self.leg_force_wrench_polytope(:,5,i), self.leg_force_wrench_polytope(:,3,i)]);
                % trisurf(k1,self.leg_force_wrench_polytope(:,1,i), self.leg_force_wrench_polytope(:,5,i), self.leg_force_wrench_polytope(:,3,i),'FaceColor','green')
                P1.plot('color', 'green', 'alpha', 0.5);
                % axis equal;
                grid on;
                % view(90,90);
                view(45,30);
                xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
                title(ax(i),'Leg ',num2str(i));

            end
            sgtitle('WLAR Force Polytope');
        end
        function fig = plot_friction_polytopes(self)
            % %% friction polytope
            self = geometry_computation.compute_contact_wrench_polytope(self);


            fig = figure;
            tiledlayout(2,2);
            for i=1:4
                ax(i) = nexttile;
                P2 = Polyhedron(self.leg_friction_polytope(:,:,i));
                % plot3(self.leg_contact_wrench_polytope(:,1,i), self.leg_contact_wrench_polytope(:,2,i), self.leg_contact_wrench_polytope(:,3,i),".","Color","r","MarkerSize",10)
                % [k2,av2] = convhull([self.leg_contact_wrench_polytope(:,1,i), self.leg_contact_wrench_polytope(:,2,i), self.leg_contact_wrench_polytope(:,3,i)]);
                % trisurf(k2,self.leg_contact_wrench_polytope(:,1,i), self.leg_contact_wrench_polytope(:,2,i), self.leg_contact_wrench_polytope(:,3,i),'FaceColor','r')
                P2.plot('color', 'red', 'alpha', 0.5);
                axis equal;
                grid on;
                % view(90,90);
                view(45,30);
                xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
                title(ax(i),'Leg ',num2str(i));

            end
            sgtitle('WLAR friction Polytope');
        end
        function fig = plot_fesible_polytopes(self)
            % Plot the first convex hull
            fig = figure;
            tiledlayout(1,3);
            ax(1) = nexttile;
            self.leg_force_polytope_total_convhull.plot('color', 'green', 'alpha', 0.5);
            title(ax(1),'Force Polytope')
            xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
            axis equal
            view(45,30);

            % Plot the second convex hull
            ax(2) = nexttile;
            self.leg_friction_polytope_total_convhull.plot('color', 'red', 'alpha', 0.5);
            title(ax(2),'Friction Polytope')
            xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
            axis equal
            view(45,30);

            % Plot the intersection
            ax(3) = nexttile;
            self.feasible_wrench_polytope_total_convhull.plot('color', 'blue', 'alpha', 0.5);
            title(ax(3),'Feasible Polytope')
            xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");
            axis equal
            view(45,30);

            sgtitle('Feasible Wrench Polytope');
        end
        function self = plot_ascender_force_polytopes(self)
            self = geometry_computation.compute_ascender_wrench_polytope(self);

            fig = figure;

            self.asc_force_polytope_convhull.plot('color', 'cyan', 'alpha', 0.5);
            % axis equal;
            grid on;
            % view(90,90);
            view(45,30);
            xlabel("F_x [N]");ylabel("\tau_y [Nm]");zlabel("F_z [N]");

            title('WLAR Ascender Force Polytope');
        
        
        end
        function fig = animation_fesible_polytopes(self)
            fig = figure;

            % 첫 번째 서브플롯 설정
            subplot(1, 3, 1);
            h1 = plot(self.leg_force_polytope_total_convhull, 'color', 'green', 'alpha', 0.5);
            title('Force Polytope');
            xlabel("F_x [N]"); ylabel("\tau_y [Nm]"); zlabel("F_z [N]");
            axis equal;
            view(45, 30);

            % 두 번째 서브플롯 설정
            subplot(1, 3, 2);
            h2 = plot(self.leg_friction_polytope_total_convhull, 'color', 'red', 'alpha', 0.5);
            title('Friction Polytope');
            xlabel("F_x [N]"); ylabel("\tau_y [Nm]"); zlabel("F_z [N]");
            axis equal;
            view(45, 30);

            % 세 번째 서브플롯 설정
            subplot(1, 3, 3);
            h3 = plot(self.feasible_wrench_polytope_total_convhull, 'color', 'blue', 'alpha', 0.5);
            title('Feasible Polytope');
            xlabel("F_x [N]"); ylabel("\tau_y [Nm]"); zlabel("F_z [N]");
            axis equal;
            view(45, 30);

            sgtitle('Feasible Wrench Polytope');

            for iteration = 0:2.5:90
                % 로봇의 각도 업데이트
                self.q.hr = [deg2rad(iteration); deg2rad(-iteration); deg2rad(-iteration); deg2rad(iteration)];

                % 기구학 계산
                self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
                self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

                % 야코비안 계산
                self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);

                % 폴리토프 계산
                self = geometry_computation.compute_force_wrench_polytope(self);
                self = geometry_computation.compute_contact_wrench_polytope(self);

                % 두 폴리토프의 교차 계산
                P1 = Polyhedron(self.leg_force_polytope_total_convhull);
                P2 = Polyhedron(self.leg_friction_polytope_total_convhull);

                % Compute the intersection of the two polyhedra
                self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);

                % 첫 번째 서브플롯 업데이트
                subplot(1, 3, 1);
                cla;
                P1.plot('color', 'green', 'alpha', 0.5);
                title('Force Polytope');
                xlabel("F_x [N]"); ylabel("\tau_y [Nm]"); zlabel("F_z [N]");
                axis equal;
                view(45, 30);

                % 두 번째 서브플롯 업데이트
                subplot(1, 3, 2);
                cla;
                P2.plot('color', 'red', 'alpha', 0.5);
                title('Friction Polytope');
                xlabel("F_x [N]"); ylabel("\tau_y [Nm]"); zlabel("F_z [N]");
                axis equal;
                view(45, 30);

                % 세 번째 서브플롯은 변화가 없는 self.feasible_wrench_polytope_total_convhull을 그리도록 설정
                subplot(1, 3, 3);
                cla;
                self.feasible_wrench_polytope_total_convhull.plot('color', 'blue', 'alpha', 0.5);
                title('Feasible Polytope');
                xlabel("F_x [N]"); ylabel("\tau_y [Nm]"); zlabel("F_z [N]");
                axis equal;
                view(45, 30);

                sgtitle('Feasible Wrench Polytope');

                % 애니메이션 효과를 위한 일시 정지
                pause(0.5);
            end
        end
    end
end

