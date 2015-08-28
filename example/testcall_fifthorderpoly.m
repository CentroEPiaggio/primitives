clear all;close all;clc;
qi=100;
qf=30;
T = 13.7;
ti = 0;
tf = ti+T;
% [coeff] = fifthorderpoly(qi,qf,ti,tf);
[a5,a4,a3,a2,a1,a0] = createTraj5(qi,qf,0,0,0,0,ti,tf);
coeff=[a5,a4,a3,a2,a1,a0];
coeffp=polyder(coeff);
coeffpp=polyder(coeffp);
t = linspace(ti,tf,1000);
figure
y=polyval(coeff,t-ti)
plot(t,y)
grid on
figure
plot(t,polyval(coeffp,t-ti))
grid on
figure
plot(t,polyval(coeffpp,t-ti))
grid on
