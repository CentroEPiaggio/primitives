%CHOOSEPARENT determines the best parent in the cost sense
% function idx_min = ChooseParent(idX_near, idx_nearest, T, G, x_new, cost_x_new)
function [idx_min,q,cost_new_edge] = ChooseParent(idX_near, idx_nearest, T, G, E, x_new, cost_from_x_nearest_to_new,Obstacles)
% make the sparse matrix square
% G = full(Graph)
sizeG = size(G);
[~,shorterDim]=min(sizeG);
G(sizeG(shorterDim)+1:max(sizeG),:)=0;
disp('Inside ChooseParent')

idx_min = idx_nearest; % default initialization

%CHOOSEPARENT determines the best parent in the cost sense
idx_I = 1; % initial node is node 1
% x_new = T.get(idx_new);
x_nearest = T.get(idx_nearest);
c_new = Inf;
cost_new_edge = Inf;

% costo = costo accumulato + costo nuovo campione
c_x_nearest = graphshortestpath(G,idx_I,idx_nearest); % calculates the optimal cost from the first node to the nearest one\
cost_z_new = cost_from_x_nearest_to_new;
c_min = c_x_nearest + cost_z_new;

% keyboard

for i=1:length(idX_near) % for every point btw the nearby vertices
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
                % cost up to near vertex
%                 cost_up_to_z_near = G(idx_I,idX_near(i));%graphshortestpath(G,idx_I,idX_near(i));
                cost_up_to_z_near = graphshortestpath(G,idx_I,idX_near(i)); % TODO this line or the one above?
%                 cost_up_to_z_near = graphshortestpath(G,idx_I,idx_);
                % costo from near to new vertex
                cost_znear_znew  = cost_new_edge;
%                 c_x_new = graphshortestpath(G,idx_I,idx_new);
                % costo fino al near + pezzettino near-new
                c_prime = (cost_up_to_z_near + cost_znear_znew < cost_z_new);
                if (c_prime < cost_z_new) && (c_prime < c_min) % cost_z_near) && % && (c_actual < c_x_new)
                    idx_min = idX_near(i);
                    c_min = c_prime;
                end
            end
        end
    end
end
end
