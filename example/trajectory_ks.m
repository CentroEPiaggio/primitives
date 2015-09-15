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
function [time,traj,amax,amin] = trajectory_ks(xi,xf,xpi,xpf,T,Ts,amax_desired,Tmax)
[a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,xpi,xpf,0,0,0,T);
coeff=[a5,a4,a3,a2,a1,a0];
coeffp=polyder(coeff);
time=0:Ts:T;
% traj = polyval(coeffp,time);
traj = polyval(coeffp,time);
% returns amax and amin as extra params for diagnostic
t_amax=-(2*a4 + 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
t_amin=-(2*a4 - 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
coeffpp = polyder(coeffp);
amax = polyval(coeffpp,t_amax);
amin = polyval(coeffpp,t_amin);

% if nargin==7 % we got a constraint on maximum (modulus) acceleration
% rescaling of the trajectory to comply with maximum acceleration
% constraints
jj=0;
while(abs(amax)>abs(amax_desired)) && jj<100
    jj=jj+1;
    T = 1.1*T;
    [a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,xpi,xpf,0,0,0,T);
    coeff=[a5,a4,a3,a2,a1,a0];
    coeffp=polyder(coeff);
    time=0:Ts:T;
    % traj = polyval(coeffp,time);
    traj = polyval(coeffp,time);
    t_amax=-(2*a4 + 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
    t_amin=-(2*a4 - 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
    coeffpp=polyder(coeffp);
    amax = polyval(coeffpp,t_amax);
    amin = polyval(coeffpp,t_amin);
    %     keyboard
end

% elseif nargin==8 % ok, so you also want a maximum time for your trajectory? No problema!
if nargin==8 % ok, so you also want a maximum time for your trajectory? No problema!
    %     xf = xf*Tmax/T;
%     keyboard
    % rescale coordinates in a linear fashion
    xf = xf*Tmax/T;
    xpf = xpf*Tmax/T;
    [a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,xpi,xpf,0,0,0,Tmax);
    coeff=[a5,a4,a3,a2,a1,a0];
    coeffp=polyder(coeff);
    time=0:Ts:Tmax;
    % traj = polyval(coeffp,time);
    traj = polyval(coeffp,time);
    t_amax=-(2*a4 + 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
    t_amin=-(2*a4 - 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
    coeffpp=polyder(coeffp);
    amax = polyval(coeffpp,t_amax);
    amin = polyval(coeffpp,t_amin);
    jj=0;
    while (abs(amax)>abs(amax_desired)) && jj<100
        jj=jj+1;
        xf= 0.9*xf;
        xpf = 0.9*xpf;
        [a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,xpi,xpf,0,0,0,Tmax);
        coeff=[a5,a4,a3,a2,a1,a0];
        coeffp=polyder(coeff);
        time=0:Ts:Tmax;
        % traj = polyval(coeffp,time);
        traj = polyval(coeffp,time);
        t_amax=-(2*a4 + 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
        t_amin=-(2*a4 - 2^(1/2)*(2*a4^2 - 5*a3*a5)^(1/2))/(10*a5);
        coeffpp=polyder(coeffp);
        amax = polyval(coeffpp,t_amax);
        amin = polyval(coeffpp,t_amin);
    end
end

if jj>=100
    time=NaN;
    traj=NaN;
end