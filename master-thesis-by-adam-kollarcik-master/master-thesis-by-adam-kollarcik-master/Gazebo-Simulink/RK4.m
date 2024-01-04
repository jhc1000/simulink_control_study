function y = RK4(delta_t,y0,k_spring,kang,u0,u3)
k1 = delta_t*ODE_fixed_torque(0,y0,[k_spring;kang],[u0;u3]);
k2 = delta_t*ODE_fixed_torque(0,y0+k1/2,[k_spring;kang],[u0;u3]);
k3 = delta_t*ODE_fixed_torque(0,y0+k2/2,[k_spring;kang],[u0;u3]);
k4 = delta_t*ODE_fixed_torque(0,y0+k3,[k_spring;kang],[u0;u3]);
y = y0 + (k1+2*k2+2*k3+k4)/6;
end

