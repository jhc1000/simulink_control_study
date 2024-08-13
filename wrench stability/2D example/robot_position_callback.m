function robot_position_callback(message)
global current_robot_position
global self

current_robot_position = [message.pose.position.x;message.pose.position.y;message.pose.position.z];
% disp(current_robot_position);

self.p_base = current_robot_position;

end

