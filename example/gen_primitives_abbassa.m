function [time,traj_y_cart,traj_yp_cart,retval]=gen_primitives_abbassa(prim_data)
%% Extract data from struct
yi = prim_data.yi;
yf = prim_data.yf;
ypi = prim_data.ypi;
ypf = prim_data.ypf;
% Tend = prim_data.Tend;
Ts = prim_data.Ts;
time = NaN;
traj_yp_cart = NaN;

% [time,traj_y_cart] = trajectory(yi,yf,ypi,ypf,Tend,Ts);

vmax = 100;
amax = 0.5;
jerkmax = 0.5;

x0 = [yi;ypi;0];
xf = [yf;ypf;0];
state_bounds = [0;vmax;amax];
control_bounds = [-jerkmax;jerkmax];
% keyboard
% [time,traj_yp_cart] = min_jerk_trajectory(x0,xf,Ts,state_bounds,control_bounds);
[time,traj_y_cart,traj_yp_cart,~,~,retval] = min_jerk_trajectory_analytic(x0,xf,Ts,state_bounds,control_bounds);

time = time(:);
traj_y_cart = traj_y_cart(:);
traj_yp_cart = traj_yp_cart(:);