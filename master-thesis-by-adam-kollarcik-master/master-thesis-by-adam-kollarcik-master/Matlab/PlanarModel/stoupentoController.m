function [u] = stoupentoController(t,y,K,P,D,equilibrium,ang00)

th0 = y(4,:);
th1 = y(5,:);
th2 = y(6,:);
th3 = y(7,:);
th4 = y(8,:);
v1 = y(1,:);
v2 = y(2,:);
v3 = y(3,:);


% 
th3ref = equilibrium(3);
% if  t<2
%     th3ref = equilibrium(3) -0.2;
% elseif t>2 && t<4
%     th3ref = equilibrium(3) +0.2;
% else
%     th3ref = equilibrium(3);
% end

ang0 =(pi/2 - angle((cos(th1 + th2)*47i)/250 - (47*sin(th1 + th2))/250 + (cos(th1)*19i)/100 - (19*sin(th1))/100 - 2^(1/2)*cos(th1 + th2 + th3)*(1/40 - 1i/40) - 2^(1/2)*sin(th1 + th2 + th3)*(1/40 + 1i/40)))-ang00+deg2rad(5);
ang0_dot = -(36972.*v2 + 35720.*v2.*cos(th2) + 4700.*2^(1/2).*v2.*cos(th3) - 4700.*2^(1/2).*v2.*sin(th3) + (1825973.*v3.*sin(th2 + th3 - th4))./(94.*sin(th3)) - 4825.*2^(1/2).*v3.*sin(th2 + th3 - th4) + 4750.*2^(1/2).*v2.*cos(th2 + th3) - (120625.*v3.*(188.*sin(th2 - th4) + 93.*sin(th2 + th3 - th4)))./(8742.*sin(th3)) - 4750.*2^(1/2).*v2.*sin(th2 + th3) + (4825.*2^(1/2).*v3.*(188.*sin(th2 - th4) + 93.*sin(th2 + th3 - th4)))/186 + (18335.*v3.*sin(th2 + th3 - th4).*cos(th2))./sin(th3) + (458375.*2^(1/2).*v3.*sin(th2 + th3).*(188.*sin(th2 - th4) + 93.*sin(th2 + th3 - th4)))./(17484.*sin(th3)) - (4825.*2^(1/2).*v3.*cos(th3).*(188.*sin(th2 - th4) + 93.*sin(th2 + th3 - th4)))./(186.*sin(th3)) + (458375.*2^(1/2).*v3.*sin(th2 + th3 - th4).*cos(th2 + th3))./(188.*sin(th3)) - (458375.*2^(1/2).*v3.*sin(th2 + th3 - th4).*sin(th2 + th3))./(188.*sin(th3)) + (4825.*2^(1/2).*v3.*sin(th2 + th3 - th4).*cos(th3))./sin(th3) - (458375.*2^(1/2).*v3.*cos(th2 + th3).*(188.*sin(th2 - th4) + 93.*sin(th2 + th3 - th4)))./(17484.*sin(th3)))./(9500.*cos(th2 + th3 + pi/4) + 35720.*cos(th2) + 9400.*cos(th3 + pi/4) + 36972);

th_dot3 =  -(193*v3.*(188*sin(th2 - th4) + 93*sin(th2 + th3 - th4)))./(17484*sin(th3));
u3 =  P*(th3ref-th3) + D*th_dot3;
u0 = -K*[v1;ang0_dot;th0;ang0];
u = [max(min(u0,2),-2) ;max(min(u3,10),-10)];






end

