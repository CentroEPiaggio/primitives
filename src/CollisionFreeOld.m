function [feasible,cost,q,x,time] = CollisionFreeOld(prim,Obstacles,q,x,time,z_begin,costo)
keyboard
% collision checker loop
% checks for feasibility of the returned steering function and, if
% not feasible, looks for an intermediate point that might be able
% to satisfy feasibility in the sense of no collisions along the
% path
% NOTE: the function ObstacleFree does more or less the same thing. Don't
% remember why, but in the end use just one of it.
feasible = true; % NOTE: default is true because it is intended that we always check for feasibility of the arguments traj_pos and traj_vel before calling this function.
cost = costo;
% keyboard
%
if size(x,1)>2
    traj_pos = x(1,:);
    traj_vel = x(2,:);
    traj_y = x(3,:);
else
%     keyboard
    traj_vel = z_begin(2)*ones(1,size(x,2));%x(2,:);
    traj_pos = z_begin(1)+cumtrapz(time,traj_vel);  
    if isnan(z_begin(3))
        traj_y = 1*ones(1,size(x,2)); % HARDFIX, has to be parametrized
    else
        traj_y = z_begin(3)*ones(1,size(x,2)); % HARDFIX, has to be parametrized
    end
end


kk=1;
nested_feasible = false; nested_cost = Inf; nested_q = q; nested_traj_pos = traj_pos; nested_traj_vel = traj_vel;
% for ii=1:Obstacles.nnodes
%         obstaclefree = any(Obstacles.Node{ii}.P.contains([nested_traj_pos(:)'; nested_traj_vel(:)']));
% keyboard
obstaclefree_move = any(Obstacles.Node{1}.P.contains([traj_pos(:)'; traj_vel(:)'],1));
obstaclefree_elevate = any(Obstacles.Node{2}.P.contains([traj_pos(:)'; traj_y(:)'],1));
obstaclefree = obstaclefree_move & obstaclefree_elevate;
    disp('meh')
while ~obstaclefree && kk<=9 % tries only 10 times to shring the trajectory
    % ~nested_feasible &&
    %     q = [xi xf vi vf];
    %     xi = q(1);
    %     vi = q(2);
    %     xf = q(3);
    %     vf = q(4);
    z_start = z_begin;
    if strcmp(prim.name,'Muovi')
%         z_start = [x(1,1);x(2,1)];
        z_end = [x(1,end);x(2,end);z_start(3);NaN];
    elseif strcmp(prim.name,'Eleva')
%         z_start = [x(1,1)];
        z_end = [z_start(1);z_start(2);x(1,end);NaN];
    end
    alpha = 1-0.1*kk; % decreases of 10 percent along the line connecting initial and final point
%     keyboard
    z_point = (1-alpha)*z_start+(alpha)*z_end;
    %     [nested_feasible,nested_cost,nested_q,nested_traj_pos,nested_traj_vel]=steering_muovi(xi,point(1),vi,point(2));
    %     [nested_feasible,nested_cost,nested_q,nested_traj_pos,nested_traj_vel]
%     keyboard
    [nested_feasible,nested_cost,nested_q,nested_x,nested_time]=prim.steering(z_start,z_point);%steering_muovi(xi,point(1),vi,point(2));
    %                 x_new=x_rand;
    %     x_new(1) = nested_traj_pos(end);%q(2);
    %     x_new(2) = nested_traj_vel(end);%q(4);
    %                 keyboard
    % assign output
    feasible = nested_feasible;
    if feasible
        cost = nested_cost;
        q = nested_q;
        time = nested_time;
        % prepare for next loop
        if strcmp(prim.name,'Muovi')
            %             [feasible,cost,q,traj_pos,traj_vel]=CollisionFree(Obstacles,q,traj_pos,traj_vel,cost);
            traj_pos = nested_x(1,:);
            traj_vel = nested_x(2,:);
            if length(z_start) < 3 || ~isnan(z_start(3)) % HARDFIX
                traj_y   = ones(size(traj_vel)); % HARDFIX: default y is 1
            else
                traj_y   = z_start(3,:)*ones(size(traj_vel));
            end
        elseif strcmp(prim.name,'Eleva') % Eleva primitive
            %             [feasible,cost,q,x]=CollisionFree(prim,Obstacles,q,x,cost);
            traj_vel = z_start(2)*ones(size(nested_x));%x(2,:);
            traj_pos = z_start(1)+cumtrapz(time,traj_vel);
            traj_y = nested_x;
        end
        x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)']; % assign arc-path % row vectors
    end
    obstaclefree_move = any(Obstacles.Node{1}.P.contains([traj_pos(:)'; traj_vel(:)']));
    obstaclefree_elevate = any(Obstacles.Node{2}.P.contains([traj_pos(:)'; traj_y(:)']));
    obstaclefree = obstaclefree_move & obstaclefree_elevate;
    
    kk = kk+1;
end
% end
% keyboard
x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)'];