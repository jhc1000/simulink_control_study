function wheelleg_joint_state_callback(message)
global joint_state
global self

joint_state = [-message.position(1),message.position(2),-message.position(3);
    -message.position(4),-message.position(5),message.position(6);
    -message.position(7),message.position(8),-message.position(9);
    -message.position(10),-message.position(11),message.position(12)];
% disp(joint_state);

self.q.hr = deg2rad(joint_state(:,1));
self.q.hp = deg2rad(joint_state(:,2));
self.q.k = deg2rad(joint_state(:,3));

%% figure
% % Check if the figure already exists
% try
%     figName = 'WLAR ForwardKinematics Simulation';
%     fig = findobj('Type', 'figure', 'Name', figName);
% 
%     if isempty(fig)
%         % If the figure doesn't exist, create it
%         fig = figure('Name', figName);
%         h = plot3(0, 0, 0);
%         pt = get(h, 'Parent');
%         xlim(pt, 'manual');
%         xlim(pt, [-10 0]);
%         ylim(pt, 'manual');
%         ylim(pt, [-5 0]);
%         zlim(pt, 'manual');
%         zlim(pt, [-1 2]);
%         axis equal;
%         % view(45, 45);
%         title('WLAR ForwardKinematics Simulation');
%         hold on;
% 
%         % Initialize lines and quivers
%         self.hPlot.L1 = gobjects(1, 4);
%         self.hPlot.L2 = gobjects(1, 4);
%         self.hPlot.L3 = gobjects(1, 4);
%         self.hPlot.Lb = gobjects(1, 4);
%         self.hPlot.Lw = gobjects(1, 4);
%         self.hPlot.L_rope = gobjects(1, 2);
%         self.hPlot.quivers = gobjects(1, width(self.p.s_ej));
%         self.hPlot.P1 = gobjects(1, 4);
%         self.hPlot.P2 = gobjects(1, 4);
%         self.hPlot.P3 = [];
%         self.hPlot.P0 = [];
% 
%         % Create the initial lines and quivers
%         for i = 1:4
%             point1.x{i} = self.p.s_hr(1,i); point1.y{i} = self.p.s_hr(2,i); point1.z{i} = self.p.s_hr(3,i);
%             point2.x{i} = self.p.s_hp(1,i); point2.y{i} = self.p.s_hp(2,i); point2.z{i} = self.p.s_hp(3,i);
%             point3.x{i} = self.p.s_k(1,i); point3.y{i} = self.p.s_k(2,i); point3.z{i} = self.p.s_k(3,i);
%             point4.x{i} = self.p.s_w(1,i); point4.y{i} = self.p.s_w(2,i); point4.z{i} = self.p.s_w(3,i);
% 
%             self.hPlot.L1(i) = line([point4.x{i}, point4.x{i}], [point4.y{i}, point4.y{i}], [point4.z{i}, point4.z{i}], 'Color', 'r', 'LineWidth', 2);
%             self.hPlot.L2(i) = line([point4.x{i}, point4.x{i}], [point4.y{i}, point4.y{i}], [point4.z{i}, point4.z{i}], 'Color', 'b', 'LineWidth', 2);
%             self.hPlot.L3(i) = line([point4.x{i}, point4.x{i}], [point4.y{i}, point4.y{i}], [point4.z{i}, point4.z{i}], 'Color', 'g', 'LineWidth', 2);
%         end
% 
%         for i = 1:2
%             plot3(self.p.s_ej(1, i), self.p.s_ej(2, i), self.p.s_ej(3, i), '.', 'Color', 'magenta', 'MarkerSize', 10);
%             plot3(self.anchor.position(1, 1, i), self.anchor.position(2, 1, i), self.anchor.position(3, 1, i), '.', 'Color', 'magenta', 'MarkerSize', 10);
%             self.hPlot.L_rope(i) = line([self.p.s_ej(1, i), self.anchor.position(1, 1, i)], [self.p.s_ej(2, i), self.anchor.position(2, 1, i)], [self.p.s_ej(3, i), self.anchor.position(3, 1, i)], 'Color', 'magenta', 'LineWidth', 3);
%         end
% 
%         % for i = 1:4
%         %     self.hPlot.P1(i) = Polyhedron(self.p.s_w(:, i).' + 0.0005 .* self.leg_actuation_force_polytope(:, :, i));
%         %     self.hPlot.P1(i).plot('color', 'green', 'alpha', 0.3);
%         % end
%         % 
%         % for i = 1:4
%         %     self.hPlot.P2(i) = Polyhedron(self.p.s_w(:, i).' + 0.00025 .* self.leg_contact_force_polytope(:, :, i));
%         %     self.hPlot.P2(i).plot('color', 'red', 'alpha', 0.3);
%         % end
%         % 
%         % self.hPlot.P3 = Polyhedron(((self.p.s_ej(:,1) + self.p.s_ej(:,2))./2).' + 0.00025 .* self.ascender_force_polytope);
%         % self.hPlot.P3.plot('color', 'magenta', 'alpha', 0.3);
% 
%         plot3([self.p_base(1)+0.115, self.com_xy_position(1)], [self.p_base(2)+0.004, self.com_xy_position(2)], [self.p_base(3), self.com_xy_position(3)], '--', 'Color', 'cyan', 'LineWidth', 3);
%         plot3(self.com_xy_position(1), self.com_xy_position(2), self.com_xy_position(3), '.', 'Color', 'cyan', 'Markersize', 20);
%         plot3(self.com_position_lp_results(1,:), self.com_position_lp_results(2,:), self.com_position_lp_results(3,:), '.', 'Color', 'red', 'Markersize', 20);
% 
%         % self.hPlot.P0 = Polyhedron(self.com_position_lp_results');
%         % self.hPlot.P0.plot('color', 'red', 'alpha', 0.3);
%         % 
%         % p = self.p.s_w(:, self.c_bool);
%         % for i = 1:width(p)
%         %     self.hPlot.quivers(i) = quiver3(p(1, i), p(2, i), p(3, i), 0.001 * self.grf((3*i-2)), 0.001 * self.grf((3*i-1)), 0.001 * self.grf((3*i)), '-', 'Color', 'r', 'LineWidth', 5);
%         % end
%         % 
%         % ej = self.p.s_ej;
%         % for i = 1:width(ej)
%         %     quiver3(ej(1,i), ej(2,i), ej(3,i), 0.1*self.tension_lp(i)*self.v_norm(1,i), 0.1*self.tension_lp(i)*self.v_norm(2,i), 0.1*self.tension_lp(i)*self.v_norm(3,i), '--', 'Color', 'k', 'LineWidth', 5);
%         % end
% 
%         hold off;
%     else
%         % If the figure exists, update it with new data
%         figure(fig);
% 
%         % Update lines
%         for i = 1:4
%             set(self.hPlot.L1(i), 'ZData', [self.p.s_hr(3, i), self.p.s_hp(3, i)], 'YData', [self.p.s_hr(2, i), self.p.s_hp(2, i)], 'XData', [self.p.s_hr(1, i), self.p.s_hp(1, i)]);
%             set(self.hPlot.L2(i), 'ZData', [self.p.s_hp(3, i), self.p.s_k(3, i)], 'YData', [self.p.s_hp(2, i), self.p.s_k(2, i)], 'XData', [self.p.s_hp(1, i), self.p.s_k(1, i)]);
%             set(self.hPlot.L3(i), 'ZData', [self.p.s_k(3, i), self.p.s_w(3, i)], 'YData', [self.p.s_k(2, i), self.p.s_w(2, i)], 'XData', [self.p.s_k(1, i), self.p.s_w(1, i)]);
%         end
% 
%         % Update Polyhedrons (similar logic as before)
%         % Delete previous and plot new polyhedrons
%         % (P1, P2, P3, and P0 updates should follow here)
% 
%         % Update Quivers
%         % p = self.p.s_w(:, self.c_bool);
%         % for i = 1:width(p)
%         %     set(self.hPlot.quivers(i), 'XData', p(1, i), 'YData', p(2, i), 'ZData', p(3, i), ...
%         %         'UData', 0.001 * self.grf((3*i-2)), 'VData', 0.001 * self.grf((3*i-1)), 'WData', 0.001 * self.grf((3*i)));
%         % end
% 
%         % Update other elements as needed (e.g., ZMP, COM position, etc.)
%     end
% 
%     xlabel('$\it{x} \rm{[m]}$', 'Interpreter', 'latex');
%     ylabel('$\it{y} \rm{[m]}$', 'Interpreter', 'latex');
%     zlabel('$\it{z} \rm{[m]}$', 'Interpreter', 'latex');
%     pause(0.1);
% catch exception
%     disp(exception)
%     clear all
% end
end

