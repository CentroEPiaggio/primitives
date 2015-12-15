close all
x0 = [0;0;0];
xf = [1;0;0];
x0 = [rand(2,1);0];
xf = [rand(2,1);0];
Ts = 0.01;
state_bounds = [Inf Inf; Inf Inf; -1 1]*200;
control_bounds = [-1; 1]*200;
min_jerk_trajectory(x0,xf,Ts,state_bounds,control_bounds)

min_jerk_trajectory_nopos(x0,xf,Ts,state_bounds,control_bounds)