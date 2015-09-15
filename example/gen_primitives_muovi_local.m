% returns: trajectory derivative (e.g. speed)
function [time,traj_vel_cart,q]=gen_primitives_muovi_local(prim_data)
%% Extract data from struct
xi = prim_data.xi;
xf = prim_data.xf;
xpi = prim_data.vi;%0;
xpf = prim_data.vf;%0;

Tend = prim_data.Tend;
Ts = prim_data.Ts;
% keyboard
amax = 10;
% [time,traj_x_cart] = trajectory(xi,xf,xpi,xpf,Tend,Ts);
% [time,traj_vel_cart] = trajectory_ks(xi,xf,xpi,xpf,Tend,Ts,amax);
[time,traj_vel_cart] = trajectory_ks(xi,xf,xpi,xpf,Tend,Ts,amax);
time = time(:)';
traj_vel_cart = traj_vel_cart(:)';

% keyboard
% restrict the trajectory to be local
% Tmax = 100; % local trajectory by restricting the integration time
% [time,traj_vel_cart] = trajectory_ks(xi,xf,xpi,xpf,Tend,Ts,amax,Tmax);


q = [xi xf xpi xpf];