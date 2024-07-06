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

            xlabel('$\it{x} \rm{[m]}$', 'Interpreter', 'latex');
            ylabel('$\it{y} \rm{[m]}$', 'Interpreter', 'latex');
            zlabel('$\it{z} \rm{[m]}$', 'Interpreter', 'latex');
        end
        function fig = plot_robot_space(self)
            fig = figure;
            h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
            xlim(pt,[-1 1]);ylim(pt,'manual');ylim(pt,[-1 1]);zlim(pt,'manual');
            zlim(pt,[-1 1]);axis equal;
            % grid on;
            % view(90,90);
            view(45,45);
            title('WLAR ForwardKinematics Simulation')
            hold on
            for i = 1:4
                point1.x{i} = self.p.s_hr(1,i); point1.y{i} = self.p.s_hr(2,i); point1.z{i} = self.p.s_hr(3,i);
                point2.x{i} = self.p.s_hp(1,i); point2.y{i} = self.p.s_hp(2,i); point2.z{i} = self.p.s_hp(3,i);
                point3.x{i} = self.p.s_k(1,i); point3.y{i} = self.p.s_k(2,i); point3.z{i} = self.p.s_k(3,i);
                point4.x{i} = self.p.s_w(1,i); point4.y{i} = self.p.s_w(2,i); point4.z{i} = self.p.s_w(3,i);
                % L1{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','r','LineWidth',2);
                % L2{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','b','LineWidth',2);
                % L3{i} = line([point4.x{i}, point4.x{i}],[point4.y{i},point4.y{i}],[point4.z{i},point4.z{i}],'Color','g','LineWidth',2);
                % set(L1{i},'ZData',[point1.z{i},point2.z{i}],'YData',[point1.y{i},point2.y{i}],'XData',[point1.x{i},point2.x{i}]);
                % set(L2{i},'ZData',[point2.z{i},point3.z{i}],'YData',[point2.y{i},point3.y{i}],'XData',[point2.x{i},point3.x{i}]);
                % set(L3{i},'ZData',[point3.z{i},point4.z{i}],'YData',[point3.y{i},point4.y{i}],'XData',[point3.x{i},point4.x{i}]);
            end
            % Lb{1} = line([point1.x{1}, point1.x{2}],[point1.y{1},point1.y{2}],[point1.z{1},point1.z{2}],'Color','cyan','LineWidth',3);
            % Lb{2} = line([point1.x{2}, point1.x{4}],[point1.y{2},point1.y{4}],[point1.z{2},point1.z{4}],'Color','b','LineWidth',3);
            % Lb{3} = line([point1.x{3}, point1.x{4}],[point1.y{3},point1.y{4}],[point1.z{3},point1.z{4}],'Color','b','LineWidth',3);
            % Lb{4} = line([point1.x{3}, point1.x{1}],[point1.y{3},point1.y{1}],[point1.z{3},point1.z{1}],'Color','b','LineWidth',3);
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
                P1 = Polyhedron(self.p.s_w(:,i).'+0.001.*self.leg_actuation_force_polytope(:,:,i));
                P1.plot('color', 'green', 'alpha', 0.5);
            end

            for i=1:4
                P2 = Polyhedron(self.p.s_w(:,i).' + 0.000025.*self.leg_contact_force_polytope(:,:,i));
                P2.plot('color', 'red', 'alpha', 0.5);
            end

            P3 = Polyhedron(((self.p.s_ej(:,1)+self.p.s_ej(:,2))./2).' + 0.0005.*self.ascender_force_polytope);
            P3.plot('color', 'cyan', 'alpha', 0.5);
            hold off

            xlabel('$\it{x} \rm{[m]}$', 'Interpreter', 'latex');
            ylabel('$\it{y} \rm{[m]}$', 'Interpreter', 'latex');
            zlabel('$\it{z} \rm{[m]}$', 'Interpreter', 'latex');
        end
        function fig = plot_force_polytopes(self)
            self = geometry_computation.compute_force_wrench_polytope(self);

            fig = figure;
            tiledlayout(2,2);
            for i=1:4
                ax(i) = nexttile;
                P1 = Polyhedron([self.leg_actuation_wrench_polytope(:,1,i) self.leg_actuation_wrench_polytope(:,5,i) self.leg_actuation_wrench_polytope(:,3,i)]);
                P1.plot('color', 'green', 'alpha', 0.5);
                % axis equal;
                grid on;
                % view(90,90);
                view(-45,30);
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
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
                P2 = Polyhedron([self.leg_contact_wrench_polytope(:,1,i) self.leg_contact_wrench_polytope(:,5,i) self.leg_contact_wrench_polytope(:,3,i)]);
                P2.plot('color', 'red', 'alpha', 0.5);
                % axis equal;
                grid on;
                % view(90,90);
                view(-45,30);
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                title(ax(i),'Leg ',num2str(i));

            end
            sgtitle('WLAR friction Polytope');
        end
        function fig = plot_fesible_polytopes(self)
            % Plot the first convex hull
            fig = figure;
            tiledlayout(1,3);
            ax(1) = nexttile;
            self.leg_actuation_wrench_polytope_total_convhull.plot('color', 'green', 'alpha', 0.5);
            title(ax(1),'Force Polytope')
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            % axis equal
            view(-45,30);

            % Plot the second convex hull
            ax(2) = nexttile;
            self.leg_contact_wrench_polytope_total_convhull.plot('color', 'red', 'alpha', 0.5);
            title(ax(2),'Friction Polytope')
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            % axis equal
            view(-45,30);

            % Plot the intersection
            ax(3) = nexttile;
            self.feasible_wrench_polytope_total_convhull.plot('color', 'blue', 'alpha', 0.5);
            title(ax(3),'Feasible Polytope')
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            % axis equal
            view(-45,30);

            sgtitle('Feasible Wrench Polytope');
        end
        function fig = plot_fesible_polytopes1(self)
            % Plot the first convex hull
            fig = figure;
            tiledlayout(1,3);
            ax(1) = nexttile;
            self.force_polytope_total_convhull.plot('color', 'green', 'alpha', 0.5);
            title(ax(1),'Force Polytope')
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            % axis equal
            % xlim([-10000 10000]); ylim([-10000 10000]); zlim([-10000 10000]);
            view(-45,30);

            % Plot the second convex hull
            ax(2) = nexttile;
            self.leg_contact_wrench_polytope_total_convhull.plot('color', 'red', 'alpha', 0.5);
            title(ax(2),'Friction Polytope')
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            % axis equal
            % xlim([-3000 3000]); ylim([-3000 3000]); zlim([-0 10000]);
            view(-45,30);

            % Plot the intersection
            ax(3) = nexttile;
            self.feasible_wrench_polytope_total1_convhull.plot('color', 'blue', 'alpha', 0.5);
            title(ax(3),'Feasible Polytope')
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            % axis equal;
            % xlim([-3000 3000]); ylim([-3000 3000]); zlim([-0 10000]);
            view(-45,30);

            sgtitle('Feasible Wrench Polytope');
        end
        function self = plot_ascender_force_polytopes(self)
            self = geometry_computation.compute_ascender_wrench_polytope(self);

            fig = figure;

            P = Polyhedron(self.ascender_force_polytope);

            % self.asc_force_polytope_convhull.plot('color', 'cyan', 'alpha', 0.5);
            P.plot('color', 'cyan', 'alpha', 0.5);
            axis equal;
            grid on;
            % view(90,90);
            view(-45,30);
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\F_y} \rm{[N]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');

            title('WLAR Ascender Force Polytope');


        end
        function fig = animation_fesible_polytopes(self)
            fig = figure;

            % 첫 번째 서브플롯 설정
            subplot(1, 3, 1);
            h1 = plot(self.leg_actuation_wrench_polytope_total_convhull, 'color', 'green', 'alpha', 0.5);
            title('Force Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-10000 10000]); ylim([-10000 10000]); zlim([-10000 10000]);
            view(-45, 30);

            % 두 번째 서브플롯 설정
            subplot(1, 3, 2);
            h2 = plot(self.leg_contact_wrench_polytope_total_convhull, 'color', 'red', 'alpha', 0.5);
            title('Friction Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-10000 10000]); ylim([-3000 3000]); zlim([-0 10000]);
            view(-45, 30);

            % 세 번째 서브플롯 설정
            subplot(1, 3, 3);
            h3 = plot(self.feasible_wrench_polytope_total_convhull, 'color', 'blue', 'alpha', 0.5);
            title('Feasible Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-10000 10000]); ylim([-3000 3000]); zlim([-0 10000]);
            view(-45, 30);

            sgtitle('Feasible Wrench Polytope');

            for iteration = 0:5:90
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
                P1 = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
                P2 = Polyhedron(self.leg_contact_wrench_polytope_total_3d);

                % Compute the intersection of the two polyhedra
                self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);

                % 첫 번째 서브플롯 업데이트
                subplot(1, 3, 1);
                cla;
                P1.plot('color', 'green', 'alpha', 0.5);
                title('Force Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-10000 10000]); ylim([-10000 10000]); zlim([-10000 10000]);
                view(-45, 30);

                % 두 번째 서브플롯 업데이트
                subplot(1, 3, 2);
                cla;
                P2.plot('color', 'red', 'alpha', 0.5);
                title('Friction Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-5000 5000]); ylim([-5000 5000]); zlim([-0 10000]);
                view(-45, 30);

                % 세 번째 서브플롯은 변화가 없는 self.feasible_wrench_polytope_total_convhull을 그리도록 설정
                subplot(1, 3, 3);
                cla;
                self.feasible_wrench_polytope_total_convhull.plot('color', 'blue', 'alpha', 0.5);
                title('Feasible Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-5000 5000]); ylim([-5000 5000]); zlim([-0 10000]);
                view(-45, 30);

                sgtitle('Feasible Wrench Polytope');

                % 애니메이션 효과를 위한 일시 정지
                pause(0.5);
            end
        end
        function fig = animation_fesible_polytopes1(self)
            fig = figure;


            % 첫 번째 서브플롯 설정
            subplot(1, 3, 1);
            h1 = plot(self.force_polytope_total_convhull, 'color', 'green', 'alpha', 0.5);
            title('Force Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-10000 10000]); ylim([-10000 10000]); zlim([-10000 10000]);
            view(45, 30);

            % 두 번째 서브플롯 설정
            subplot(1, 3, 2);
            h2 = plot(self.leg_contact_wrench_polytope_total_convhull, 'color', 'red', 'alpha', 0.5);
            title('Friction Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-5000 5000]); ylim([-5000 5000]); zlim([-0 10000]);
            view(45, 30);

            % 세 번째 서브플롯 설정
            subplot(1, 3, 3);
            h3 = plot(self.feasible_wrench_polytope_total1_convhull, 'color', 'blue', 'alpha', 0.5);
            title('Feasible Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-5000 5000]); ylim([-5000 5000]); zlim([-0 10000]);
            view(45, 30);

            sgtitle('Feasible Wrench Polytope');

            for iteration = 0:5:90
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
                self = geometry_computation.compute_ascender_wrench_polytope(self);

                self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d, self.asc_wrench_polytope_3d);

                % Create polyhedra from vertices
                P1 = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
                P2 = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
                P3 = Polyhedron(self.force_polytope_total_3d);

                % Compute the intersection of the two polyhedra
                self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);
                self.feasible_wrench_polytope_total1_convhull = intersect(P3, P2);

                % 첫 번째 서브플롯 업데이트
                subplot(1, 3, 1);
                cla;
                P3.plot('color', 'green', 'alpha', 0.5);
                title('Force Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-10000 10000]); ylim([-10000 10000]); zlim([-10000 10000]);
                view(45, 30);

                % 두 번째 서브플롯 업데이트
                subplot(1, 3, 2);
                cla;
                P2.plot('color', 'red', 'alpha', 0.5);
                title('Friction Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-5000 5000]); ylim([-5000 5000]); zlim([-0 10000]);
                view(45, 30);

                % 세 번째 서브플롯은 변화가 없는 self.feasible_wrench_polytope_total_convhull을 그리도록 설정
                subplot(1, 3, 3);
                cla;
                self.feasible_wrench_polytope_total1_convhull.plot('color', 'blue', 'alpha', 0.5);
                title('Feasible Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-5000 5000]); ylim([-5000 5000]); zlim([-0 10000]);
                view(45, 30);

                sgtitle('Feasible Wrench Polytope');

                % 애니메이션 효과를 위한 일시 정지
                pause(0.5);
            end
        end
        function fig = animation_fk_sim(self)
            self.q_base = [0.0; deg2rad(0); 0.0];
            self.p_base = [-2.0; -1.972; 0.4594];

            self.dot_q_base = [0.0; 0.0; 0.0];
            self.dot_p_base = [0.0; 0.0; 0.0];

            self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
            self.q.hp = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
            self.q.k = [deg2rad(90); deg2rad(90); deg2rad(-90); deg2rad(-90)];

            % 기구학 계산
            self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
            self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

            % 야코비안 계산
            self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);

            % 폴리토프 계산
            self = geometry_computation.compute_force_wrench_polytope(self);
            self = geometry_computation.compute_contact_wrench_polytope(self);
            self = geometry_computation.compute_ascender_wrench_polytope(self);

            self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d, self.asc_wrench_polytope_3d);
            self.force_polytope_total_convhull = Polyhedron(self.force_polytope_total_3d);

            % Create polyhedra from vertices
            P1 = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
            P2 = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
            P3 = Polyhedron(self.force_polytope_total_3d);

            % Compute the intersection of the two polyhedra
            self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);
            self.feasible_wrench_polytope_total1_convhull = intersect(P3, P2);

            fig = figure;
            % 첫 번째 서브플롯 설정
            subplot(1,4,1);
            h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
            xlim(pt,[-1 1]);ylim(pt,'manual');ylim(pt,[-1 1]);zlim(pt,'manual');
            zlim(pt,[-1 1]);axis equal;
            grid on;
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
                P1 = Polyhedron(self.p.s_w(:,i).'+0.001.*self.leg_actuation_force_polytope(:,:,i));
                P1.plot('color', 'green', 'alpha', 0.5);
            end

            for i=1:4
                P2 = Polyhedron(self.p.s_w(:,i).' + 0.000025.*self.leg_contact_force_polytope(:,:,i));
                P2.plot('color', 'red', 'alpha', 0.5);
            end

            P3 = Polyhedron(((self.p.s_ej(:,1)+self.p.s_ej(:,2))./2).' + 0.0005.*self.ascender_force_polytope);
            P3.plot('color', 'cyan', 'alpha', 0.5);
            hold off

            xlabel('$\it{x} \rm{[m]}$', 'Interpreter', 'latex');
            ylabel('$\it{y} \rm{[m]}$', 'Interpreter', 'latex');
            zlabel('$\it{z} \rm{[m]}$', 'Interpreter', 'latex');


            % 두 번째 서브플롯 설정
            subplot(1, 4, 2);
            h1 = plot(self.force_polytope_total_convhull, 'color', 'green', 'alpha', 0.5);
            title('Force Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-7500 7500]); ylim([-7500 7500]); zlim([-7500 7500]);
            view(-45, 30);

            % 세 번째 서브플롯 설정
            subplot(1, 4, 3);
            h2 = plot(self.leg_contact_wrench_polytope_total_convhull, 'color', 'red', 'alpha', 0.5);
            title('Friction Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-7500 7500]); ylim([-7500 7500]); zlim([-0 7500]);
            view(-45, 30);

            % 네 번째 서브플롯 설정
            subplot(1, 4, 4);
            h3 = plot(self.feasible_wrench_polytope_total1_convhull, 'color', 'blue', 'alpha', 0.5);
            title('Feasible Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-7500 7500]); ylim([-7500 7500]); zlim([-0 7500]);
            view(-45, 30);

            for iteration = 0:5:90
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
                self = geometry_computation.compute_ascender_wrench_polytope(self);

                self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d, self.asc_wrench_polytope_3d);
                self.force_polytope_total_convhull = Polyhedron(self.force_polytope_total_3d);

                % Create polyhedra from vertices
                P1 = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
                P2 = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
                P3 = Polyhedron(self.force_polytope_total_3d);

                % Compute the intersection of the two polyhedra
                self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);
                self.feasible_wrench_polytope_total1_convhull = intersect(P3, P2);

                % 첫 번째 서브플롯 설정
                subplot(1,4,1);
                cla;
                h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
                xlim(pt,[-1 1]);ylim(pt,'manual');ylim(pt,[-1 1]);zlim(pt,'manual');
                zlim(pt,[-1 1]);axis equal;
                grid on;
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
                    P1 = Polyhedron(self.p.s_w(:,i).'+0.001.*self.leg_actuation_force_polytope(:,:,i));
                    P1.plot('color', 'green', 'alpha', 0.5);
                end

                for i=1:4
                    P2 = Polyhedron(self.p.s_w(:,i).' + 0.000025.*self.leg_contact_force_polytope(:,:,i));
                    P2.plot('color', 'red', 'alpha', 0.5);
                end

                P3 = Polyhedron(((self.p.s_ej(:,1)+self.p.s_ej(:,2))./2).' + 0.0005.*self.ascender_force_polytope);
                P3.plot('color', 'cyan', 'alpha', 0.5);
                hold off

                xlabel('$\it{x} \rm{[m]}$', 'Interpreter', 'latex');
                ylabel('$\it{y} \rm{[m]}$', 'Interpreter', 'latex');
                zlabel('$\it{z} \rm{[m]}$', 'Interpreter', 'latex');


                % 첫 번째 서브플롯 설정
                subplot(1, 4, 2);
                cla;
                h1 = plot(self.force_polytope_total_convhull, 'color', 'green', 'alpha', 0.5);
                title('Force Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-7500 7500]); ylim([-7500 7500]); zlim([-7500 7500]);
                view(-45, 30);

                % 두 번째 서브플롯 설정
                subplot(1, 4, 3);
                cla;
                h2 = plot(self.leg_contact_wrench_polytope_total_convhull, 'color', 'red', 'alpha', 0.5);
                title('Friction Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-7500 7500]); ylim([-7500 7500]); zlim([-0 7500]);
                view(-45, 30);

                % 세 번째 서브플롯 설정
                subplot(1, 4, 4);
                cla;
                h3 = plot(self.feasible_wrench_polytope_total1_convhull, 'color', 'blue', 'alpha', 0.5);
                title('Feasible Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-7500 7500]); ylim([-7500 7500]); zlim([-0 7500]);
                view(-45, 30);

                % 애니메이션 효과를 위한 일시 정지
                pause(0.5);
            end
        end
        function fig = animation_fk_sim_xyz(self)
            self.q_base = [0.0; deg2rad(0); 0.0];
            % self.p_base = [-2.0; -1.972; 0.4594];
            self.p_base = [-2.0; -3.0; 0.4594];

            self.dot_q_base = [0.0; 0.0; 0.0];
            self.dot_p_base = [0.0; 0.0; 0.0];

            self.q.hr = [deg2rad(0); deg2rad(-0); deg2rad(-0); deg2rad(0)];
            self.q.hp = [deg2rad(-45); deg2rad(-45); deg2rad(45); deg2rad(45)];
            self.q.k = [deg2rad(90); deg2rad(90); deg2rad(-90); deg2rad(-90)];

            % 기구학 계산
            self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
            self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

            % 야코비안 계산
            self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);

            % 폴리토프 계산
            self = geometry_computation.compute_force_wrench_polytope(self);
            self = geometry_computation.compute_contact_wrench_polytope(self);
            self = geometry_computation.compute_ascender_wrench_polytope(self);

            self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d, self.asc_wrench_polytope_3d);
            self.force_polytope_total_convhull = Polyhedron(self.force_polytope_total_3d);

            % Create polyhedra from vertices
            P1 = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
            P2 = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
            P3 = Polyhedron(self.force_polytope_total_3d);

            % Compute the intersection of the two polyhedra
            self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);
            self.feasible_wrench_polytope_total1_convhull = intersect(P3, P2);

            fig = figure;
            % 첫 번째 서브플롯 설정
            subplot(1,4,1);
            h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
            xlim(pt,[-1 1]);ylim(pt,'manual');ylim(pt,[-1 1]);zlim(pt,'manual');
            zlim(pt,[-1 1]);axis equal;
            grid on;
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
                P1 = Polyhedron(self.p.s_w(:,i).'+0.001.*self.leg_actuation_force_polytope(:,:,i));
                P1.plot('color', 'green', 'alpha', 0.5);
            end

            for i=1:4
                P2 = Polyhedron(self.p.s_w(:,i).' + 0.000025.*self.leg_contact_force_polytope(:,:,i));
                P2.plot('color', 'red', 'alpha', 0.5);
            end

            P3 = Polyhedron(((self.p.s_ej(:,1)+self.p.s_ej(:,2))./2).' + 0.0005.*self.ascender_force_polytope);
            P3.plot('color', 'cyan', 'alpha', 0.5);
            hold off

            xlabel('$\it{x} \rm{[m]}$', 'Interpreter', 'latex');
            ylabel('$\it{y} \rm{[m]}$', 'Interpreter', 'latex');
            zlabel('$\it{z} \rm{[m]}$', 'Interpreter', 'latex');


            % 첫 번째 서브플롯 설정
            subplot(1, 4, 2);
            h1 = plot(self.force_polytope_total_convhull, 'color', 'green', 'alpha', 0.5);
            title('Force Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-7500 7500]); ylim([-7500 7500]); zlim([-7500 7500]);
            view(-45, 30);

            % 두 번째 서브플롯 설정
            subplot(1, 4, 3);
            h2 = plot(self.leg_contact_wrench_polytope_total_convhull, 'color', 'red', 'alpha', 0.5);
            title('Friction Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-7500 7500]); ylim([-7500 7500]); zlim([-0 7500]);
            view(-45, 30);

            % 세 번째 서브플롯 설정
            subplot(1, 4, 4);
            h3 = plot(self.feasible_wrench_polytope_total1_convhull, 'color', 'blue', 'alpha', 0.5);
            title('Feasible Polytope');
            xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
            ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
            zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
            axis equal;
            xlim([-7500 7500]); ylim([-7500 7500]); zlim([-0 7500]);
            view(-45, 30);

            for iteration = 0:0.1:2
                % 로봇의 각도 업데이트
                % self.q.hr = [deg2rad(iteration); deg2rad(-iteration); deg2rad(-iteration); deg2rad(iteration)];
                self.p_base = [-2.0; -3.0+iteration; 0.4594];
                % self.p_base = [-2.0-iteration; -1.972; 0.4594];

                % 기구학 계산
                self = self.kinematics.forward_kinematics(self, self.q_base, self.p_base);
                self = self.kinematics.ascender_forward_kinematics(self, self.q_base, self.p_base);

                % 야코비안 계산
                self = self.kinematics.Jacobians(self, self.dot_q_base, self.dot_p_base);

                % 폴리토프 계산
                self = geometry_computation.compute_force_wrench_polytope(self);
                self = geometry_computation.compute_contact_wrench_polytope(self);
                self = geometry_computation.compute_ascender_wrench_polytope(self);

                self.force_polytope_total_3d = geometry_computation.minkowskiSum(self.leg_actuation_wrench_polytope_total_3d, self.asc_wrench_polytope_3d);
                self.force_polytope_total_convhull = Polyhedron(self.force_polytope_total_3d);

                % Create polyhedra from vertices
                P1 = Polyhedron(self.leg_actuation_wrench_polytope_total_3d);
                P2 = Polyhedron(self.leg_contact_wrench_polytope_total_3d);
                P3 = Polyhedron(self.force_polytope_total_3d);

                % Compute the intersection of the two polyhedra
                self.feasible_wrench_polytope_total_convhull = intersect(P1, P2);
                self.feasible_wrench_polytope_total1_convhull = intersect(P3, P2);

                % 첫 번째 서브플롯 설정
                subplot(1,4,1);
                cla;
                h = plot3(0,0,0);pt = get(h,'Parent');xlim(pt,'manual');
                xlim(pt,[-1 1]);ylim(pt,'manual');ylim(pt,[-1 1]);zlim(pt,'manual');
                zlim(pt,[-1 1]);axis equal;
                grid on;
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
                    P1 = Polyhedron(self.p.s_w(:,i).'+0.001.*self.leg_actuation_force_polytope(:,:,i));
                    P1.plot('color', 'green', 'alpha', 0.5);
                end

                for i=1:4
                    P2 = Polyhedron(self.p.s_w(:,i).' + 0.000025.*self.leg_contact_force_polytope(:,:,i));
                    P2.plot('color', 'red', 'alpha', 0.5);
                end

                P3 = Polyhedron(((self.p.s_ej(:,1)+self.p.s_ej(:,2))./2).' + 0.0005.*self.ascender_force_polytope);
                P3.plot('color', 'cyan', 'alpha', 0.5);
                hold off

                xlabel('$\it{x} \rm{[m]}$', 'Interpreter', 'latex');
                ylabel('$\it{y} \rm{[m]}$', 'Interpreter', 'latex');
                zlabel('$\it{z} \rm{[m]}$', 'Interpreter', 'latex');


                % 두 번째 서브플롯 설정
                subplot(1, 4, 2);
                cla;
                h1 = plot(self.force_polytope_total_convhull, 'color', 'green', 'alpha', 0.5);
                title('Force Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-7500 7500]); ylim([-7500 7500]); zlim([-7500 7500]);
                view(-45, 30);

                % 세 번째 서브플롯 설정
                subplot(1, 4, 3);
                cla;
                h2 = plot(self.leg_contact_wrench_polytope_total_convhull, 'color', 'red', 'alpha', 0.5);
                title('Friction Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-7500 7500]); ylim([-7500 7500]); zlim([-0 7500]);
                view(-45, 30);

                % 네 번째 서브플롯 설정
                subplot(1, 4, 4);
                cla;
                h3 = plot(self.feasible_wrench_polytope_total1_convhull, 'color', 'blue', 'alpha', 0.5);
                title('Feasible Polytope');
                xlabel('$\it{F_x} \rm{[N]}$', 'Interpreter', 'latex');
                ylabel('$\it{\tau_y} \rm{[Nm]}$', 'Interpreter', 'latex');
                zlabel('$\it{F_z} \rm{[N]}$', 'Interpreter', 'latex');
                axis equal;
                xlim([-7500 7500]); ylim([-7500 7500]); zlim([-0 7500]);
                view(-45, 30);

                % 애니메이션 효과를 위한 일시 정지
                pause(0.25);
            end
        end
    end
end

