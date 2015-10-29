function [feasible,cost,q,x,time] = CollisionFree(prim,Obstacles,q,x,time,z_begin,costo)
% keyboard
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
traj_pos = x(1,:);
traj_vel = x(2,:);
traj_y = x(3,:);

kk=1;
obstaclefree=true;
for ii=1:Obstacles.nnodes
    if ii==1
        checkon = [traj_pos(:)'; traj_vel(:)'];
    elseif ii==2
        checkon = [traj_pos(:)'; traj_y(:)'];
    else
        error('remember to extend it')
    end
    obstaclefree=obstaclefree & any(Obstacles.Node{1}.P.contains(checkon,1));
end
if obstaclefree
    disp('obstaclefree')
    feasible = true;
    cost = costo;
else
    disp('in collision')
    feasible = false;
    cost = Inf;
end
return
%% FIX THIS LOOP
z_start = z_begin;
if strcmp(prim.name,'Muovi')
    %         z_start = [x(1,1);x(2,1)];
    z_end = [x(1,end);x(2,end);z_start(3);NaN];
elseif strcmp(prim.name,'Eleva')
    %         z_start = [x(1,1)];
    z_end = [z_start(1);z_start(2);x(1,end);NaN];
end
while ~obstaclefree && kk<=7 % tries only 10 times to shring the trajectory
    
    
    alpha = 1-0.1*kk; % decreases of 10 percent along the line connecting initial and final point
    z_point = (1-alpha)*z_start+(alpha)*z_end;
    [nested_feasible,nested_cost,nested_q,nested_x,nested_time]=prim.steering(z_start,z_point);%steering_muovi(xi,point(1),vi,point(2));
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
    % update while variables
    kk = kk+1;
    obstaclefree=true;
    for ii=1:Obstacles.nnodes
        if ii==1
            checkon = [traj_pos(:)'; traj_vel(:)'];
        elseif ii==2
            checkon = [traj_pos(:)'; traj_y(:)'];
        else
            error('remember to extend it')
        end
        obstaclefree=obstaclefree & any(Obstacles.Node{1}.P.contains(checkon,1));
    end
end
x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)'];