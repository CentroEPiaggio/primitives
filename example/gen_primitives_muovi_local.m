% returns: trajectory derivative (e.g. speed)
function [time,traj_vel_cart,q]=gen_primitives_muovi_local(prim_data)
%% Extract data from struct
xi = prim_data.xi;
xf = prim_data.xf;
xpi = prim_data.vi;%0;
xpf = prim_data.vf;%0;

Tend = prim_data.Tend;
Ts = prim_data.Ts;

amax = 0.5;
jerkmax = 0.5;
% [time,traj_vel_cart] = trajectory_ks(xi,xf,xpi,xpf,Tend,Ts,amax);
x0 = [xi;xpi;0];
xf = [xf;xpf;0];
state_bounds = [-Inf Inf;
    -Inf Inf;
    -amax amax];
control_bounds = [-jerkmax;jerkmax];

[time,traj_vel_cart] = min_jerk_trajectory(x0,xf,Ts,state_bounds,control_bounds);
time = time(:)';
traj_vel_cart = traj_vel_cart(:)';

q = [prim_data.xi prim_data.xf prim_data.vi prim_data.vf];