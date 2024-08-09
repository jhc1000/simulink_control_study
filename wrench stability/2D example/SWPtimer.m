function SWPtimer(~, ~, self, handles)
    
    % Update the pose message values
    if isvalid(handles.swpPub)
        handles.swpPubmsg.pose.position.x = self.base_movement(1);
        handles.swpPubmsg.pose.position.y = self.base_movement(2);
        handles.swpPubmsg.pose.position.z = self.base_movement(3);
    
        % Publish the scan and pose messages
        send(handles.swpPub,handles.swpPubmsg);
    end

    if isvalid(handles.swpolytopePub)
        tempmsg=rosmessage('geometry_msgs/Point32');
        for i = 1:width(self.com_position_lp_results)
            tempmsg.X = self.com_position_lp_results(1,i);
            tempmsg.Y = self.com_position_lp_results(2,i);
            tempmsg.Z = self.com_position_lp_results(3,i);
            handles.swpPubmsg.Polygon.Points(i) = tempmsg;
        end
    
        % Publish the scan and pose messages
        send(handles.swpolytopePub,handles.swpolytopePubmsg);
    end
end