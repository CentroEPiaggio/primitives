% returns: trajectory derivative (e.g. speed)
function [time,traj_pos_cart,traj_vel_cart,q,retval,cost]=gen_primitives_dd_muovi_local(prim_data)
%% Extract data from struct
xi = prim_data.xi;
xf = prim_data.xf;
xpi = prim_data.vi;%0;
xpf = prim_data.vf;%0;

% Tend = prim_data.Tend; % DONE now it is parameterized

%
%  x0 = [0;0;0]
% xf = [0.65;-0.58;0]
% Ts = 0.01

Ts = prim_data.Ts;

vmax = 1;
amax = 0.5;
jerkmax = 1;
% [time,traj_vel_cart] = trajectory_ks(xi,xf,xpi,xpf,Tend,Ts,amax);
x0 = [xi;xpi;0];
xf = [xf;xpf;0];
state_bounds = [-Inf Inf;
    -vmax vmax;
    -amax amax];
control_bounds = [-jerkmax;jerkmax];

[time,traj_pos_cart,traj_vel_cart,~,~,retval,cost] = dd_optimal_trajectory(x0,xf,Ts,state_bounds,control_bounds);

time = time(:)';
traj_pos_cart = traj_pos_cart(:)';
traj_vel_cart = traj_vel_cart(:)';

q = [prim_data.xi prim_data.xf prim_data.vi prim_data.vf];