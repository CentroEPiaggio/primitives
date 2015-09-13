function [time,traj_x_cart]=gen_primitives_muovi(prim_data)
%% Extract data from struct
xi = prim_data.xi;
xf = prim_data.xf;
xpi = 0;%prim_data.xpi;
xpf = 0;%prim_data.xpf;

Tend = prim_data.Tend;
Ts = prim_data.Ts;

[time,traj_x_cart] = trajectory(xi,xf,xpi,xpf,Tend,Ts);
time = time(:)';
traj_x_cart = traj_x_cart(:)';
