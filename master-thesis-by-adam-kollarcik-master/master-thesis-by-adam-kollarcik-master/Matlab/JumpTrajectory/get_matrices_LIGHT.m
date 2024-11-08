function [M,C,N,B,J] = get_matrices_LIGHT(in1,in2)
%GET_MATRICES_LIGHT
%    [M,C,N,B,J] = GET_MATRICES_LIGHT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    25-Dec-2020 13:03:43

k_ang = in2(2,:);
kspring = in2(1,:);
th1 = in1(9,:);
th2 = in1(10,:);
th3 = in1(11,:);
th4 = in1(12,:);
v2 = in1(4,:);
v3 = in1(5,:);
t2 = cos(th1);
t3 = cos(th2);
t4 = cos(th3);
t5 = cos(th4);
t6 = sin(th1);
t7 = sin(th2);
t8 = sin(th3);
t9 = sin(th4);
t10 = th1+th2;
t11 = th1+th4;
t18 = -th4;
t21 = pi./4.0;
t12 = t8.^2;
t13 = t8.^3;
t14 = cos(t10);
t15 = cos(t11);
t16 = sin(t10);
t17 = sin(t11);
t19 = 1.0./t8;
t22 = t18+th2;
t26 = t21+th3;
t39 = t2.*(3.71e+2./1.0e+3);
t40 = t6.*(3.71e+2./1.0e+3);
t47 = t3.*5.7152e-2;
t48 = t7.*5.7152e-2;
t53 = t5.*2.316e-3;
t20 = 1.0./t13;
t23 = cos(t22);
t24 = sin(t22);
t25 = t22+th3;
t29 = cos(t26);
t30 = sin(t26);
t31 = t26+th2;
t34 = t10+t26;
t42 = -t39;
t43 = -t40;
t45 = t14.*(1.88e+2./6.25e+2);
t46 = t16.*(1.88e+2./6.25e+2);
t50 = t15.*9.65e-3;
t52 = t17.*9.65e-3;
t58 = t16.*2.950848;
t66 = t17.*9.46665e-2;
t27 = cos(t25);
t28 = sin(t25);
t32 = cos(t31);
t33 = sin(t31);
t35 = t24.^2;
t36 = cos(t34);
t37 = sin(t34);
t41 = t24.*1.88e+2;
t49 = -t45;
t51 = -t46;
t54 = -t50;
t55 = -t52;
t61 = t29.*2.82e-2;
t64 = t29.*1.41e-2;
t65 = t30.*1.41e-2;
t67 = -t66;
t38 = t28.^2;
t44 = t28.*9.3e+1;
t56 = t36.*(3.0./4.0e+1);
t57 = t37.*(3.0./4.0e+1);
t62 = t32.*1.425e-2;
t63 = t33.*1.425e-2;
t68 = t37.*7.3575e-1;
t72 = t64+1.56125e-2;
t59 = -t56;
t60 = -t57;
t69 = t41+t44;
t70 = t45+t56;
t71 = t46+t57;
t73 = t48+t63;
t74 = t63+t65;
t75 = t62+t72;
t88 = t47+t61+t62+7.098476666666667e-2;
t76 = t19.*t36.*t69.*8.278997940974605e-4;
t77 = t19.*t37.*t69.*8.278997940974605e-4;
t78 = t19.*t28.*t70.*(1.93e+2./1.88e+2);
t79 = t19.*t28.*t71.*(1.93e+2./1.88e+2);
t82 = t19.*t28.*t73.*v3.*(1.93e+2./1.88e+2);
t83 = t43+t51+t55+t60;
t84 = t42+t49+t54+t59;
t85 = t19.*t69.*t74.*v3.*1.103866392129947e-2;
t86 = t19.*t69.*t75.*1.103866392129947e-2;
t89 = t19.*t28.*t88.*(1.93e+2./1.88e+2);
t80 = -t78;
t81 = -t79;
t87 = -t86;
t90 = t54+t76+t80;
t91 = t55+t77+t81;
t92 = t53+t87+t89+1.241633333333333e-3;
M = reshape([2.3e+1./1.0e+1,0.0,0.0,t84,t90,0.0,2.3e+1./1.0e+1,0.0,t83,t91,0.0,0.0,7.350000000000001e-4,0.0,0.0,t84,t83,0.0,t3.*1.14304e-1+t5.*4.632e-3+t32.*2.85e-2+t61+1.431964e-1,t92,t90,t91,0.0,t92,t19.*t28.*(t19.*t28.*(t61+7.098476666666667e-2).*(1.93e+2./1.88e+2)-t19.*t69.*t72.*1.103866392129947e-2).*(1.93e+2./1.88e+2)+t19.*t69.*(t19.*t69.*1.72341140471288e-4-t19.*t28.*t72.*(1.93e+2./1.88e+2)).*1.103866392129947e-2+1.241633333333333e-3],[5,5]);
if nargout > 1
    C = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./1.0e+2,1.0./1.0e+2,0.0,t40.*v2+t46.*v2+t52.*v2+t52.*v3+t57.*v2+t16.*t19.*t28.*v3.*(1.93e+2./6.25e+2)-t19.*t24.*t37.*v3.*1.556451612903226e-1,t2.*v2.*(-3.71e+2./1.0e+3)-t14.*v2.*(1.88e+2./6.25e+2)-t15.*v2.*9.65e-3-t15.*v3.*9.65e-3-t36.*v2.*(3.0./4.0e+1)-t14.*t19.*t28.*v3.*(1.93e+2./6.25e+2)+t19.*t24.*t36.*v3.*1.556451612903226e-1,1.0./1.0e+2,-t82+t85-t9.*v3.*2.316e-3+1.0./1.0e+2,t19.*v2.*(t8.*t9.*-3.72e+2-t7.*t28.*9.424e+3+t24.*t30.*4.7e+3+t24.*t33.*4.75e+3+t28.*t30.*2.325e+3).*(-6.225806451612903e-6),t20.*(t13.*t17.*v2.*1.35501e+5+t13.*t17.*v3.*1.35501e+5+t12.*t14.*t27.*v3.*4.336032e+6+t12.*t16.*t28.*v2.*4.336032e+6-t4.*t14.*t38.*v3.*4.451352e+6+t8.*t16.*t38.*v3.*4.451352e+6-t12.*t23.*t36.*v3.*2.1855e+6-t12.*t24.*t37.*v2.*2.1855e+6+t4.*t35.*t36.*v3.*4.5355e+6+t8.*t35.*t37.*v3.*4.5355e+6-t4.*t14.*t24.*t28.*v3.*8.998432e+6+t8.*t14.*t24.*t27.*v3.*8.998432e+6+t4.*t24.*t28.*t36.*v3.*2.243625e+6+t8.*t23.*t28.*t36.*v3.*2.243625e+6).*7.121718658902886e-8,t20.*(t13.*t15.*v2.*1.35501e+5+t13.*t15.*v3.*1.35501e+5+t12.*t14.*t28.*v2.*4.336032e+6-t12.*t16.*t27.*v3.*4.336032e+6+t4.*t16.*t38.*v3.*4.451352e+6+t8.*t14.*t38.*v3.*4.451352e+6-t12.*t24.*t36.*v2.*2.1855e+6+t12.*t23.*t37.*v3.*2.1855e+6-t4.*t35.*t37.*v3.*4.5355e+6+t8.*t35.*t36.*v3.*4.5355e+6+t4.*t16.*t24.*t28.*v3.*8.998432e+6-t8.*t16.*t24.*t27.*v3.*8.998432e+6-t4.*t24.*t28.*t37.*v3.*2.243625e+6-t8.*t23.*t28.*t37.*v3.*2.243625e+6).*(-7.121718658902886e-8),0.0,t75.*(t19.*v3.*(t23.*1.88e+2+t27.*9.3e+1-1.93e+2).*1.103866392129947e-2-t4.*t20.*t69.^2.*v3.*1.218521011673987e-4)-t9.*(v2.*2.316e-3+v3.*2.316e-3)-t19.*t28.*(t82-t85+t73.*v2).*(1.93e+2./1.88e+2)-t20.*t88.*v3.*(t12.*t27.*1.7484e+4-t4.*t38.*1.7949e+4-t4.*t24.*t28.*3.6284e+4+t8.*t24.*t27.*3.6284e+4).*5.871629745372061e-5+(t69.*(t30.*9.4e+1+t33.*9.5e+1).*(t8.*v2.*9.3e+1-t24.*v3.*1.93e+2).*1.780429664725722e-8)./t12,(1.0./t12.^2.*(t12.*t35.*1.036646984927898e+34+t12.*t38.*2.53676996736119e+33+t12.^2.*2.407033684834865e+33+t4.*t24.^3.*v3.*1.679375082296803e+34+t4.*t28.^3.*v3.*7.210125983832451e+33+t12.*t24.*t28.*5.128094127568857e+33-t12.*t23.*t24.*v3.*8.092325526093404e+33+t4.*t24.*t38.*v3.*1.457530844043549e+34+t4.*t28.*t35.*v3.*8.307546949659718e+33-t12.*t27.*t28.*v3.*7.023335155235755e+33+t8.*t23.*t24.*t28.*v3.*8.307546949659718e+33-t8.*t24.*t27.*t28.*v3.*1.457530844043549e+34+t12.*t23.*t28.*t29.*v3.*3.615306359936044e+33+t12.*t24.*t27.*t29.*v3.*3.615306359936044e+33-t4.*t24.*t29.*t38.*v3.*7.422916249655921e+33-t4.*t28.*t29.*t35.*v3.*1.500546510683132e+34-t8.*t23.*t29.*t38.*v3.*3.71145812482796e+33+t8.*t27.*t29.*t35.*v3.*7.502732553415662e+33-t8.*t24.*t30.*t38.*v3.*3.71145812482796e+33-t8.*t28.*t30.*t35.*v3.*7.502732553415662e+33))./1.203516842417433e+35],[5,5]);
end
if nargout > 2
    N = [0.0;2.2563e+1;0.0;t6.*(-3.63951)-t58+t67-t68;t67-t19.*t28.*(t58+t68+(kspring.*(k_ang.*2.0-th2.*2.0))./2.0).*(1.93e+2./1.88e+2)+t19.*t37.*t69.*8.121696980096088e-3];
end
if nargout > 3
    B = reshape([0.0,0.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,t19.*t69.*(-1.103866392129947e-2)],[5,2]);
end
if nargout > 4
    J = reshape([1.0,0.0,0.0,1.0,-7.0./1.0e+2,0.0,0.0,0.0,0.0,0.0],[2,5]);
end
