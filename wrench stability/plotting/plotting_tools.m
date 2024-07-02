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
                [k1,av1] = convhull(0.000025.*self.leg_force_polytope(:,:,i));
                trisurf(k1,0.00025.*self.leg_force_polytope(:,1,i)+self.p.s_w(1,i),0.00025.*self.leg_force_polytope(:,2,i)+self.p.s_w(2,i),0.00025.*self.leg_force_polytope(:,3,i)+self.p.s_w(3,i),'FaceColor','green')
            end

            for i=1:4
                [k2,av2] = convhull(0.000025.*self.leg_friction_polytope(:,:,i));
                trisurf(k2,0.000025.*self.leg_friction_polytope(:,1,i)+self.p.s_w(1,i),0.000025.*self.leg_friction_polytope(:,2,i)+self.p.s_w(2,i),0.000025.*self.leg_friction_polytope(:,3,i)+self.p.s_w(3,i),'FaceColor','r')
            end
            hold off
            
            xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');        
        end
        function fig = plot_force_polytopes(self)
            % %% force polytope
            % self.leg_torque_space(:,:,1) = [self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
            %                             -self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
            %                             self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
            %                             self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
            %                             -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) self.model.LF_tau_lim(3);
            %                             self.model.LF_tau_lim(1) -self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
            %                             -self.model.LF_tau_lim(1) self.model.LF_tau_lim(2) -self.model.LF_tau_lim(3);
            %                             -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(1) -self.model.LF_tau_lim(3)];
            % 
            % self.leg_torque_space(:,:,2) = [self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
            %                             -self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
            %                             self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
            %                             self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
            %                             -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) self.model.RF_tau_lim(3);
            %                             self.model.RF_tau_lim(1) -self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
            %                             -self.model.RF_tau_lim(1) self.model.RF_tau_lim(2) -self.model.RF_tau_lim(3);
            %                             -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(1) -self.model.RF_tau_lim(3)];
            % 
            % self.leg_torque_space(:,:,3) = [self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
            %                             -self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
            %                             self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
            %                             self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
            %                             -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) self.model.LH_tau_lim(3);
            %                             self.model.LH_tau_lim(1) -self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
            %                             -self.model.LH_tau_lim(1) self.model.LH_tau_lim(2) -self.model.LH_tau_lim(3);
            %                             -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(1) -self.model.LH_tau_lim(3)];
            % 
            % self.leg_torque_space(:,:,4) = [self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
            %                             -self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
            %                             self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
            %                             self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
            %                             -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) self.model.RH_tau_lim(3);
            %                             self.model.RH_tau_lim(1) -self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
            %                             -self.model.RH_tau_lim(1) self.model.RH_tau_lim(2) -self.model.RH_tau_lim(3);
            %                             -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(1) -self.model.RH_tau_lim(3)];
            
            figure;
            title('WLAR Force Polytope');
            for i=1:4
                % self.psuedo_inverse_jacobian(:,:,i) = (self.Jacobian_b(:,:,i)*self.Jacobian_b(:,:,i).')\self.Jacobian_b(:,:,i);
                % for j=1:8
                %     self.leg_wrench_polytope(j,:,i) = (self.psuedo_inverse_jacobian(:,:,i)*self.leg_torque_space(j,:,i).').';
                %     self.leg_force_polytope(j,:,i) = [self.leg_wrench_polytope(j,1,i) self.leg_wrench_polytope(j,2,i) self.leg_wrench_polytope(j,3,i)];
                % end
                subplot(2,2,i);
                plot3(self.leg_force_wrench_polytope(:,1,i), self.leg_force_wrench_polytope(:,2,i), self.leg_force_wrench_polytope(:,3,i),".","Color","r","MarkerSize",10)
                % plot(alphaShape(self.leg_force_polytope(:,:,i)));
                [k1,av1] = convhull(self.leg_force_polytope(:,:,i));
                trisurf(k1,self.leg_force_polytope(:,1,i),self.leg_force_polytope(:,2,i),self.leg_force_polytope(:,3,i),'FaceColor','green')
                axis equal; grid on; 
                % view(90,90);
                % view(0,0);
                xlabel("f_x");ylabel("f_y");zlabel("f_z");
               
            end
        end
            function fig = plot_friction_polytopes(self)
            % %% friction polytope
            
            
            figure;
            title('WLAR friction Polytope');
            for i=1:4
                subplot(2,2,i);
                plot3(self.leg_contact_wrench_polytope(:,1,i), self.leg_contact_wrench_polytope(:,2,i), self.leg_contact_wrench_polytope(:,3,i),".","Color","r","MarkerSize",10)
                [k2,av2] = convhull(self.leg_friction_polytope(:,:,i));
                trisurf(k2,self.leg_friction_polytope(:,1,i),self.leg_friction_polytope(:,2,i),self.leg_friction_polytope(:,3,i),'FaceColor','r')
                axis equal; grid on; 
                % view(90,90);
                % view(0,0);
                xlabel("f_x");ylabel("f_y");zlabel("f_z");
               
            end
        end
    end
end

