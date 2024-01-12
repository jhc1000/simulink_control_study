function [p] = ForwardKinematics(q)
    l1 = 200;
    l2 = 200;

    % define Rotation matrix
    for i = 1:1
        R.b_c1{i} = rotz(q.c1(i));
        R.c1_2{i} = [1 0 0;0 -1 0;0 0 -1]*rotz(q.c2(i));
        R.c2_3{i} = [-1 0 0;0 1 0;0 0 -1];
        
        p.b_c1{i} = [0; 0; 0];
        p.c1_2{i} = [l1; 0; 0];
        p.c2_3{i} = [l2; 0; 0];

        
        % Forward Kinematics
        T.b_c1{i} = TransformationMatrix(R.b_c1{i},p.b_c1{i});
        T.c1_2{i} = TransformationMatrix(R.c1_2{i},p.c1_2{i});
        T.c2_3{i} = TransformationMatrix(R.c2_3{i},p.c2_3{i});
        
        T.b_c2{i} = T.b_c1{i}*T.c1_2{i};
        p.b_c2{i} = T.b_c2{i}([1,2,3],[4]);

        T.b_c3{i} = T.b_c1{i}*T.c1_2{i}*T.c2_3{i};
        p.b_c3{i} = T.b_c3{i}([1,2,3],[4]);
        
    end
end

