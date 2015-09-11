% TRAJECTORY_KS
% kinematic scaling of 5-th order polynomial trajectories
% [time,traj] = trajectory_ks(xi,xf,xpi,xpf,T)
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
function [time,traj,amax,amin] = trajectory_ks(xi,xf,xpi,xpf,T,Ts,amax_desired)
[a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,xpi,xpf,0,0,0,T);
coeff=[a5,a4,a3,a2,a1,a0];
coeffp=polyder(coeff);
time=0:Ts:T;
traj = polyval(coeffp,time);
% returns amax and amin as extra params for diagnostic
t_amax=-(2*a4 + 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
t_amin=-(2*a4 - 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
coeffpp = polyder(coeffp);
amax = polyval(coeffpp,t_amax);
amin = polyval(coeffpp,t_amin);

% rescaling of the trajectory to comply with maximum acceleration
% constraints
while(abs(amax)>abs(amax_desired))
    T = 1.1*T;
    [a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,xpi,xpf,0,0,0,T);
    coeff=[a5,a4,a3,a2,a1,a0];
    coeffp=polyder(coeff);
    time=0:Ts:T;
    traj = polyval(coeffp,time);
    t_amax=-(2*a4 + 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
    t_amin=-(2*a4 - 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
    coeffpp=polyder(coeffp);
    amax = polyval(coeffpp,t_amax);
    amin = polyval(coeffpp,t_amin);
end
