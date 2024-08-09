function SWPtimer(~, ~, self, handles)
    
    % Update the pose message values
    if isvalid(handles.swpPub)
        handles.swpPubmsg.pose.position.x = self.base_movement(1);
        handles.swpPubmsg.pose.position.y = self.base_movement(2);
        handles.swpPubmsg.pose.position.z = self.base_movement(3);
    
        % Publish the scan and pose messages
        send(handles.swpPub,handles.swpPubmsg);
    end
end