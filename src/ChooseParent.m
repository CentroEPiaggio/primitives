%CHOOSEPARENT determines the best parent in the cost sense
% function idx_min = ChooseParent(idX_near, idx_nearest, T, G, x_new, cost_x_new)
function [idx_min,q,cost_new_edge,traj_pos,traj_vel] = ChooseParent(idX_near, idx_nearest, T, G, E, x_new, cost_from_x_nearest_to_new,Obstacles,q)
disp('Entered inside ChooseParent')
% make the sparse matrix square
% G = full(Graph)
sizeG = size(G);
[~,shorterDim]=min(sizeG);
G(sizeG(shorterDim)+1:max(sizeG),:)=0;
disp('Inside ChooseParent')

%CHOOSEPARENT determines the best parent in the cost sense
idx_I = 1; % initial node is node 1
% x_new = T.get(idx_new);
x_nearest = T.get(idx_nearest);
c_new = Inf;

% costo = costo accumulato + costo nuovo campione
cost_z_nearest = graphshortestpath(G,idx_I,idx_nearest); % calculates the optimal cost from the first node to the nearest one\
c_x_new = cost_from_x_nearest_to_new;
cost_z_new = cost_z_nearest + c_x_new;
c_min = cost_z_new; % initializazion of c_min

idx_min = idx_nearest; % default initialization row 2 algorithm 2
cost_new_edge = c_min; % default initialization row 2 algorithm 2
traj_pos = NaN;
traj_vel = NaN;
% keyboard

for i=1:length(idX_near) % for every point btw the nearby vertices
    if ~isempty(idX_near(i))
        X_near(:,i)=T.get(idX_near(i));
        % calculate feasibility and cost
        [feasible,cost_new_edge,q,traj_pos_chooseparent,traj_vel_chooseparent] = steering_muovi(X_near(1,i),x_new(1),X_near(2,i),x_new(2));
        if feasible && ~isinf(cost_new_edge) && ~isnan(cost_new_edge) % last two conditions are useless, could be probably removed without problems
            if ~any(Obstacles.Node{1}.P.contains([traj_pos_chooseparent(:)'; traj_vel_chooseparent(:)'],1)) % ObstacleFree
                % cost up to near vertex
%                 cost_up_to_z_near = G(idx_I,idX_near(i));%graphshortestpath(G,idx_I,idX_near(i));
                cost_up_to_z_near = graphshortestpath(G,idx_I,idX_near(i)); % TODO this line or the one above?
%                 cost_up_to_z_near = graphshortestpath(G,idx_I,idx_);
                % costo from near to new vertex
                cost_znear_znew  = cost_new_edge;
%                 c_x_new = graphshortestpath(G,idx_I,idx_new);
                % costo fino al near + pezzettino near-new
                c_prime = cost_up_to_z_near + cost_znear_znew;
                if (c_prime < cost_z_new) && (c_prime < c_min) % cost_z_near) && % && (c_actual < c_x_new)
                    idx_min = idX_near(i);
                    c_min = c_prime;
                    traj_pos = traj_pos_chooseparent;
                    traj_vel = traj_vel_chooseparent;
                end
            end
        end
    end
end
end
