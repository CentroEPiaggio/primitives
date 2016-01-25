% close all
%
 x0 = [0;0;0]
xf = [0.65;-0.58;0]
Ts = 0.001
% 
% x0 = [0;0;0];
% xf = [4.24;0.52;0];

% WRONG RESULT
x0 = [6.1520;0.9400;0]
xf = [5.9921;0.95;0]

vmax = 1;
amax = 1;
jerkmax = 1;

state_bounds = [-Inf Inf;
    -vmax vmax;
    -amax amax];
control_bounds = [-jerkmax;jerkmax];

[time,traj_pos_cart,traj_vel_cart,~,~,retval] = min_jerk_trajectory_analytic(x0,xf,Ts,state_bounds,control_bounds);