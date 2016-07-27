clear all; close all; clc
x0 = [0;0;0;0];
xf = [1;1;pi/2;0];
% x0 = [0.49;0.28;0.28;0.09];
% xf = [0.57;0.57;0.0;0]
% x0 = [0.49;0.28;0;0.09];
% xf = [0.79;0.28;0;0.09]
Ts = 0.01;
tic
[time,x,u,retval,cost] = dd_trajectory(x0,xf,Ts,[],[])
toc
figure
plot(x(1,:),x(2,:),'linewidth',2)
hold on
plot(x0(1),x0(2),'ko','linewidth',2)
plot(xf(1),xf(2),'ro','linewidth',2)
% [time,x,u,retval,cost] = dd_trajectory_2(x0,xf,Ts,[],[])
% plot(x(1,:),x(2,:),'m--','linewidth',2)

[time,x,u,retval,cost] = dd_trajectory_robotic(x0,xf,Ts,[],[])
plot(x(1,:),x(2,:),'r-.','linewidth',2)