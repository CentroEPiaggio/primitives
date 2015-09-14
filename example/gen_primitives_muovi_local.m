% returns: trajectory derivative (e.g. speed)
function [time,traj_x_cart]=gen_primitives_muovi_local(prim_data)
%% Extract data from struct
xi = prim_data.xi;
xf = prim_data.xf;
xpi = prim_data.vi;%0;
xpf = prim_data.vf;%0;

Tend = prim_data.Tend;
Ts = prim_data.Ts;
% keyboard
amax = 10;
[time,traj_x_cart] = trajectory(xi,xf,xpi,xpf,Tend,Ts);
% [time,traj_x_cart] = trajectory_ks(xi,xf,xpi,xpf,Tend,Ts,amax);
time = time(:)';
traj_x_cart = traj_x_cart(:)';

% restrict the trajectory to be local
% temp = time(time<=Tend);
% v_final = traj_x_cart(length(temp));
% [time,traj_x_cart] = trajectory(xi,xf,xpi,v_final,Tend,Ts);
