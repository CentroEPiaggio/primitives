% MUOVI
%
% [time,traj] = muovi(xi,xf,xpi,xpf,T)
% arguments:
% - xi:  initial coordinate of cart center of mass
% - xf:  final coordinate of cart com
% - xpi: initial velocity of ...
% - xpf: final velocity of ...
% - T:   duration of the motion
% - Ts:  sampling time of the trajectory
% returns:
% time: time vector
% traj: desired trajectory time derivative (i.e., speed)
function [time,traj] = muovi(xi,xf,xpi,xpf,T,Ts)
[a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,xpi,xpf,0,0,0,T);
coeff=[a5,a4,a3,a2,a1,a0];
coeffp=polyder(coeff);
time=0:Ts:T;
% x_cart = polyval(coeff,time);
traj = polyval(coeffp,time);