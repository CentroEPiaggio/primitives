function [time,traj_yp_cart]=gen_primitives_abbassa(prim_data)
%% Extract data from struct
yi = prim_data.yi;
yf = prim_data.yf;
ypi = prim_data.ypi;
ypf = prim_data.ypf;
Tend = prim_data.Tend;
Ts = prim_data.Ts;

% [time,traj_y_cart] = trajectory(yi,yf,ypi,ypf,Tend,Ts);

amax = 10;
jerkmax = 100;

x0 = [yi;ypi;0];
xf = [yf;ypf;0];
state_bounds = [0;0;amax];
control_bounds = [-jerkmax;jerkmax];

[time,traj_yp_cart] = min_jerk_trajectory(x0,xf,Ts,state_bounds,control_bounds);
time = time(:);
traj_yp_cart = traj_yp_cart(:);