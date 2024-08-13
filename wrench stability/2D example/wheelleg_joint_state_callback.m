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

end

