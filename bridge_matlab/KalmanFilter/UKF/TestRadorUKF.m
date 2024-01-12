close all
clear all
clc

dt = 0.05;
t = 0:dt:20;

Nsamples = length(t);

Xsaved = zeros(Nsamples,3);
Zsaved = zeros(Nsamples,1);
a = zeros(Nsamples,1);

for k = 1:Nsamples
    r = GetRadar(dt);

    [pos,vel,alt] = RadarUKF(r, dt);
    a(k) = sqrt(pos.^2+alt.^2);
    Xsaved(k,:) = [pos, vel, alt];
    Zsaved(k) = r;
end

Possaved = Xsaved(:,1);
Velsaved = Xsaved(:,2);
Altsaved = Xsaved(:,3);

t = 0:dt:Nsamples*dt-dt;

figure;
hold on
subplot(3,1,1);
plot(t,Possaved)
title('Distance')
subplot(3,1,2);
plot(t,Velsaved)
title('Velocity')
subplot(3,1,3);
plot(t,Altsaved)
title('Altitude')
hold off

figure;
hold on
plot(t,Zsaved,"b:*")
plot(t,a,"r",'LineWidth',2)
legend("measured data","UKF data")