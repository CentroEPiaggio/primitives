function [feasible,cost,q,traj_pos,traj_vel] = CollisionFree(Obstacles,q,traj_pos,traj_vel,costo)
% collision checker loop
% checks for feasibility of the returned steering function and, if
% not feasible, looks for an intermediate point that might be able
% to satisfy feasibility in the sense of no collisions along the
% path
% NOTE: the function ObstacleFree does more or less the same thing. Don't
% remember why, but in the end use just one of it.
feasible = true; % NOTE: default is true because it is intended that we always check for feasibility of the arguments traj_pos and traj_vel before calling this function.
cost = costo;

kk=1;
nested_feasible = false; nested_cost = Inf; nested_q = q; nested_traj_pos = traj_pos; nested_traj_vel = traj_vel;

while any(Obstacles.Node{1}.P.contains([nested_traj_pos(:)'; nested_traj_vel(:)'])) && kk<=10 % tries only 10 times to shring the trajectory
    % ~nested_feasible &&
    kk = kk+1;
%     q = [xi xf vi vf];
    xi = q(1);
    vi = q(2);
    xf = q(3);
    vf = q(4);
    start = [xi,vi];
    fin = [xf,vf];
    alpha = 1-0.1*kk; % decreases of 10 percent along the line connecting initial and final point
    point = (1-alpha)*start+(alpha)*fin;
    [nested_feasible,nested_cost,nested_q,nested_traj_pos,nested_traj_vel]=steering_muovi(xi,point(1),vi,point(2));
    %                 x_new=x_rand;
    x_new(1) = nested_traj_pos(end);%q(2);
    x_new(2) = nested_traj_vel(end);%q(4);
    %                 keyboard
    feasible = nested_feasible;
    cost = nested_cost;
    q = nested_q;
    traj_pos = nested_traj_pos;
    traj_vel = nested_traj_vel;
end
