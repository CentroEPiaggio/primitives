% function [time,pos,speed,acc,jerk,retval,cost] = dd_trajectory(x0,xf,Ts,state_bounds,control_bounds)
function [time,x,u,retval,cost] = arm_trajectory(x0,xf,Ts,state_bounds,control_bounds,A_g_0,q0_arm)
debug = 1;
verbose = 1;
% arm parameters
L_arm = 0.31; % maximum radius of reachability region of the arm w.r.t. base frame, i.e. sum of length of the links

% initialization
x = [];
u = [];

if verbose
    tic
end

time = [];
pos = [];
speed = [];
acc = [];
jerk = [];

% ensure vectors are column
x0 = x0(:);
xf = xf(:);

cost=Inf;

x_i=x0(1);
y_i=x0(2);
th_i=x0(3);
v_i=x0(4);
tau_i=x0(5);

x_f=xf(1);
y_f=xf(2);
% th_f=xf(3);
th_f=th_i;
% v_f=xf(4);
v_f=v_i;
% tau_f=xf(5);
tau_f=xf(3);

distance_from_goal = L_arm*(1-tau_f);

q_roomba_0 = [x_i; y_i; th_i; v_i; 0];

% if debug
%     %-keyboard
% end

[flag,time,tau,traj_q,traj_qp]=arm_trajectory_generator(Ts,q_roomba_0,A_g_0,distance_from_goal,q0_arm);
%-keyboard
x=tau;
retval = flag;
return
end