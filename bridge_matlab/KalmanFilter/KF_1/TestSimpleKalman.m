close all
clear all
clc

dt = 1;
t = 0:dt:1000;

Nsamples = length(t);

Xsaved = zeros(Nsamples, 1);
Zsaved = zeros(Nsamples, 1);

for k = 1:Nsamples
    z = getmovingvolt();
    volt = KF(z);
    
    Xsaved(k) = volt;
    Zsaved(k) = z;
end

figure
plot(t,Xsaved,'o-')
hold on
plot(t,Zsaved,'r:*')
legend('KalmanFilter','measured','Location','southeast');
axis([0 1000 0 1000])