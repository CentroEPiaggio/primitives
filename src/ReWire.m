function [ T ] = ReWire( idX_near, idx_min, idx_new, x_new, T, G, E, )
%REWIRE Summary of this function goes here
%   Detailed explanation goes here
for i=1:length(idX_near) % for every point btw the nearby vertices
    if idX_near(i)==idx_min % avoid idx_min
        continue;
    end
    if ~isempty(idX_near(i))
        X_near(:,i)=T.get(idX_near(i));
%         % remove NaNs where not needed, this part shall be extended for
%         % managing multiple primitives
%         dimensions = isnan(X_near(i,:));
%         x_near
% ... actually now with just steering_muovi we don't need any of it
        % calculate feasibility and cost
        [feasible,cost_new_edge,q,traj_pos,traj_vel] = steering_muovi(X_near(1,i),x_new(1),X_near(2,i),x_new(2));
        if feasible && ~isinf(cost_new_edge) && ~isnan(cost_new_edge) % last two conditions are useless, could be probably removed without problems
            if ~any(Obstacles.Node{1}.P.contains([traj_pos(:)'; traj_vel(:)'])) % ObstacleFree
                if cost_tentative < cost_z_near
                    keyboard
                end
                % costo fino al near
                c_x_near = graphshortestpath(G,idx_I,idX_near(i));
%                 c_x_new = graphshortestpath(G,idx_I,idx_new);
                % costo fino al near + pezzettino near-new
                c_actual = c_x_near + cost_new_edge;
                if (c_actual < c_min)% && (c_actual < c_x_new)
                    idx_min = idX_near(i);
                    c_min = c_actual;
                end
                
                % costo fino al near
                cost_z_near = G(idx_I,idX_near(i));%graphshortestpath(G,idx_I,idX_near(i));
%                 cost_up_to_z_near = graphshortestpath(G,idx_I,idx_);
                cost_znear_znew  = cost_new_edge;
%                 c_x_new = graphshortestpath(G,idx_I,idx_new);
                % costo fino al near + pezzettino near-new
                if (cost_up_to_z_near + cost_znear_znew < cost_z_near)% && (c_actual < c_x_new)
                    [T,G,E] = ReConnect(z_new,z_near,T,G,E);
                end
            end
        end
    end
end
end