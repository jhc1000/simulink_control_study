close all
clear all
clc

dt = 0.1;
t = 0:dt:10;
Nsamples = length(t);

Xsaved = zeros(Nsamples,2);
Zsaved = zeros(Nsamples,1);
real_velocity = zeros(Nsamples,1);

for k=1:Nsamples
    [x, v] = GetPos(dt);
    z = x;
    [pos, vel] = DvKalman(z,dt);

    Xsaved(k,:) = [pos, vel];
    Zsaved(k) = z;
    real_velocity(k) = v;
end

figure
plot(t,Xsaved(:,1))
hold on
plot(t,Zsaved(:,1),'r.')
legend('KalmanFilter','measured','Location','southeast');

figure
plot(t,Xsaved(:,2))
hold on
plot(t,real_velocity(:,1),'r.')
legend('KalmanFilter','measured','Location','southeast');
