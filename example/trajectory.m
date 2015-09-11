% TRAJECTORY
%
% [time,traj] = trajectory(xi,xf,xpi,xpf,T)
% arguments:
% - xi:  initial position coordinate
% - xf:  final position coordinate
% - xpi: initial velocity
% - xpf: final velocity
% - T:   duration of the motion
% - Ts:  sampling time of the trajectory
% returns:
% time: time vector
% traj: desired trajectory time derivative (i.e., speed)
% amax: maximum acceleration during trajectory
% amin: minimum acceleration during trajectory
function [time,traj,amax,amin] = trajectory(xi,xf,xpi,xpf,T,Ts)
[a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,xpi,xpf,0,0,0,T);
coeff=[a5,a4,a3,a2,a1,a0];
coeffp=polyder(coeff);
time=0:Ts:T;
% x_cart = polyval(coeff,time);
traj = polyval(coeffp,time);
% returns amax and amin as extra params
t_amax=-(2*a4 + 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
t_amin=-(2*a4 - 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
coeffpp = polyder(coeffp);
% traj_acc = polyval(coeffpp,time);
% amax = max(traj_acc);
% amin = min(traj_acc);
amax = polyval(coeffpp,t_amax);
amin = polyval(coeffpp,t_amin);
