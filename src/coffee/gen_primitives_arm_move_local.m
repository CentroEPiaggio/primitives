% returns: trajectory derivative (e.g. speed)
function [time,x,u,q,retval,cost]=gen_primitives_arm_move_local(prim_data)
%% Extract data from struct
xi = prim_data.xi;
xf = prim_data.xf;
yi = prim_data.yi;
yf = prim_data.yf;
thi = prim_data.thi;
thf = prim_data.thf;
vi = prim_data.vi;
vf = prim_data.vf;
taui = prim_data.taui;
tauf = prim_data.tauf;

% Tend = prim_data.Tend; % DONE now it is parameterized

%
%  x0 = [0;0;0]
% xf = [0.65;-0.58;0]
% Ts = 0.01

Ts = prim_data.Ts;

A_g_0 = prim_data.A_g_0; % FIXME: this has to be expressed in inertial frame

vmax = 1;

% [time,traj_vel_cart] = trajectory_ks(xi,xf,xpi,xpf,Tend,Ts,amax);
x0 = [xi;yi;thi;vi;taui];
xf = [xf;yf;thf;vf;tauf];
state_bounds = [-Inf Inf; % TODO: add joint state limits
    -Inf Inf;
    -pi pi];
control_bounds = [-vmax;vmax];

%[time,traj_pos_cart,traj_vel_cart,~,~,retval,cost] = dd_optimal_trajectory(x0,xf,Ts,state_bounds,control_bounds);
% [time,traj_pos_cart,traj_vel_cart,~,~,retval,cost] = dd_trajectory(x0,xf,Ts,state_bounds,control_bounds);
[time,x,u,retval,cost] = arm_trajectory(x0,xf,Ts,state_bounds,control_bounds,A_g_0);

%time = time(:)';
%traj_pos_cart = traj_pos_cart(:)';
%traj_vel_cart = traj_vel_cart(:)';

q = [prim_data.xi prim_data.xf prim_data.yi prim_data.yf prim_data.thi prim_data.thf];