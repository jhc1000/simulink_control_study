function handle_obstacle_overcome_command_callback(message)
global self
try
    command_str = split(message.data);
    % disp(cell2mat(command_str(2)));
    if cell2mat(command_str(1)) == 'contact'
        self.bool_contact = [str2num(cell2mat(command_str(2))), str2num(cell2mat(command_str(3))), str2num(cell2mat(command_str(4))), str2num(cell2mat(command_str(5)))];
    end
    % disp(self.bool_contact);
end
end
