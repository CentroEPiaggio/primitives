function [nest_feasible,nest_cost,nest_q,nest_traj_pos,nest_traj_vel] = ObstacleFree(nest_traj_pos, nest_traj_vel)

kk=1;
nest_feasible = false; 
nest_cost = Inf; 
nest_q = q; 
nest_traj_pos = traj_pos; 
nest_traj_vel = traj_vel;

while any(Obstacles.Node{1}.P.contains([nest_traj_pos(:)'; nest_traj_vel(:)'])) && kk<=10 % tries only 10 times to shring the trajectory
    kk = kk+1;
    q = [xi xf vi vf];
    start = [xi,vi];
    fin = [xf,vf];
    alpha = 1-0.1*kk; % decreases of 10 percent along the line connecting initial and final point
    point = (1-alpha)*start+(alpha)*fin;
    [nest_feasible,nest_cost,nest_q,nest_traj_pos,nest_traj_vel]=steering_muovi(xi,point(1),vi,point(2));

    x_new(1) = nest_traj_pos(end);%q(2);
    x_new(2) = nest_traj_vel(end);%q(4);


end

