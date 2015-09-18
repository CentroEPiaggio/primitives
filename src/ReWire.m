function [ T, G, E ] = ReWire( idX_near, idx_min, idx_new, x_new, T, G, E, prim, q, cost_new)
%REWIRE Summary of this function goes here
%   Detailed explanation goes here
idx_I = 1;

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
%         [feasible,cost_new_edge,q,traj_pos,traj_vel] = steering_muovi(X_near(1,i),x_new(1),X_near(2,i),x_new(2));
% try to reach the nearby nodes through the new node: i.e. start from the
% new node and try to reach one of the nearby nodes
        [feasible,cost_rewire,q,traj_pos,traj_vel] = steering_muovi(x_new(1),X_near(1,i),x_new(2),X_near(2,i));
        if feasible && ~isinf(cost_new_edge) && ~isnan(cost_new_edge) % last two conditions are useless, could be probably removed without problems
            if ~any(Obstacles.Node{1}.P.contains([traj_pos(:)'; traj_vel(:)'])) % ObstacleFree
                cost_up_to_z_new = cost_new;
                c_prime = cost_rewire;
                cost_tentative = cost_up_to_z_new + c_prime;
                cost_up_to_z_near = graphshortestpath(G,idx_I,idX_near(i)); % TODO: well, this should be taken from the cost graph
                if cost_tentative < cost_up_to_z_near_without_rewiring % test for correlation between sleep hours and variable length, possibly inverse proportionality
                    disp('OK, once here thou shalt rewire')
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