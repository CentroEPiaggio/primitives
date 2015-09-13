function [time,traj_y_cart]=gen_primitives_abbassa(prim_data)
%% Extract data from struct
yi = prim_data.yi;
yf = prim_data.yf;
ypi = prim_data.ypi;
ypf = prim_data.ypf;
Tend = prim_data.Tend;
Ts = prim_data.Ts;

[time,traj_y_cart] = trajectory(yi,yf,ypi,ypf,Tend,Ts);
time = time(:);
traj_y_cart = traj_y_cart(:);