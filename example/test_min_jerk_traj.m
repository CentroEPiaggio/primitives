clear all;close all;clc;

x0 = [0;0;0];
xf = [1;0;0];
x0 = [rand(2,1);0];
xf = [rand(2,1);0];
% x0 = [0;0;0];
% xf = [10;0;0];
Ts = 0.01;
state_bounds = [Inf Inf; -2 2; -1 1]*1;
state_bounds = [Inf Inf; -2 2; -1.5 1.5]*1;
V = rand;
A = rand;
D = rand;
J = rand;
% state_bounds = [Inf Inf; -2 2; -1.5 1.5]*1;
% control_bounds = [-1; 1]*2;
state_bounds = [Inf Inf; -V V; -D A]*1;
control_bounds = [-J; J]*10;
% [time,pos,speed,acc] = optimal_profile(x0,xf,Ts,state_bounds,control_bounds)
%%
x0 = -x0;
xf = -xf;
%%
Ts = 0.01;
[time_an,pos_an,speed_an,acc_an,jerk_an] = min_jerk_trajectory_analytic(x0,xf,Ts,state_bounds,control_bounds);

%%
% J=J/10;
acc = cumtrapz(time_an,jerk_an);
speed = x0(2)+cumtrapz(time_an,acc);
pos = x0(1)+cumtrapz(time_an,speed);
time = time_an;
pf = xf(1);
figure
plot(time,pos,'b',time,speed,'r',time,acc,'k',time,jerk_an,'c','linewidth',2),grid on,legend('pos','vel','acc','control jerk','location','best')
hold on
% plot(time(1),x0(1),'bo',time(end),xf(1),'bo','linewidth',2)
plot(time(1),x0(1),'bo',time(end),pf,'bo','linewidth',2)
plot(time(1),x0(2),'ro',time(end),xf(2),'ro','linewidth',2)
plot([time(1),time(end)],[V V],'r--');
plot([time(1),time(end)],[-V -V],'r--');
plot(time(1),x0(3),'ko',time(end),xf(3),'ko','linewidth',2)
plot([time(1),time(end)],[A A],'k--');
plot([time(1),time(end)],[-D -D],'k--');
plot([time(1),time(end)],[J J],'c--');
plot([time(1),time(end)],[-J -J],'c--');
title('after rearrangement')
return
[time,pos,speed,acc,jerk] = min_jerk_trajectory(x0,xf,Ts,state_bounds,control_bounds,time_an(end),jerk_an)
close all
plot(time,pos,'b',time,speed,'r',time,acc,'k',time,jerk,'c','linewidth',2),grid on,legend('pos','vel','acc','control jerk','location','best')
hold on
plot(time(1),x0(1),'bo',time(end),xf(1),'bo','linewidth',2)
plot(time(1),x0(2),'ro',time(end),xf(2),'ro','linewidth',2)
plot(time(1),x0(3),'ko',time(end),xf(3),'ko','linewidth',2)

return
% min_jerk_trajectory_nopos(x0,xf,Ts,state_bounds,control_bounds)
[x,v,a,t] = profilo_trapezoidale(x0(1),xf(1),state_bounds(2,2),state_bounds(3,2),control_bounds(2),Ts);
figure
subplot(3,1,1)
plot(time,pos,t,x),grid on
subplot(3,1,2)
plot(time,speed,t,v),grid on
subplot(3,1,3)
plot(time,acc,t,a),grid on