%CHOOSEPARENT determines the best parent in the cost sense
% function idx_min = ChooseParent(idX_near, idx_nearest, T, G, x_new, cost_x_new)
function [idx_min,q,cost_new_edge,x,time] = ChooseParent(idX_near, idx_nearest, T, G, E, z_new, cost_from_z_nearest_to_new,Obstacles,q,Ptree,idx_prim)
disp('Entered inside ChooseParent')
% make the sparse matrix square
% G = full(Graph)
sizeG = size(G);
[~,shorterDim]=min(sizeG);
G(sizeG(shorterDim)+1:max(sizeG),:)=0;
disp('Inside ChooseParent')

%CHOOSEPARENT determines the best parent in the cost sense
idx_I = 1; % initial node is node 1

% costo = costo accumulato + costo nuovo campione
cost_z_nearest = graphshortestpath(G,idx_I,idx_nearest); % calculates the optimal cost from the first node to the nearest one\
c_z_new = cost_from_z_nearest_to_new;
cost_z_new = cost_z_nearest + c_z_new;
c_min = cost_z_new; % initializazion of c_min

idx_min = idx_nearest; % default initialization row 2 algorithm 2
cost_new_edge = c_min; % default initialization row 2 algorithm 2
% traj_pos = NaN;
% traj_vel = NaN;
x = NaN;
time = NaN;
% keyboard

for i=1:length(idX_near) % for every point btw the nearby vertices
    if ~isempty(idX_near(i))
        z_near=T.get(idX_near(i));
        % select primitive HARDFIX
        if length(z_new)>=3 && z_near(3) ~= z_new(3) && ~isnan(z_new(3)) && ~isnan(z_near(3))
            prim = Ptree.Node{2}; % Elevate
            idx_prim = 2;
        else
            prim = Ptree.Node{1}; % Move
            idx_prim = 1;
        end
        %         keyboard
        % calculate feasibility and cost
        %         [feasible,cost_new_edge,q,traj_pos_chooseparent,traj_vel_chooseparent] = steering_muovi(X_near(1,i),x_new(1),X_near(2,i),x_new(2));
        %         z_near = z_near(:,i);
        %         keyboard
        [feasible,cost_new_edge,q,x_chooseparent,time_chooseparent] = prim.steering(z_near,z_new); % uniform interface! Yeay!
        if feasible
            if idx_prim == 1 % trying to fix the connection problem between different kind of primitives
                traj_pos_chooseparent = x_chooseparent(1,:);
                traj_vel_chooseparent = x_chooseparent(2,:);
                if ~isnan(z_near(3)) % HARDFIX
                    traj_y_chooseparent   = z_near(3,:)*ones(size(traj_vel_chooseparent));
                else
                    traj_y_chooseparent   = ones(size(traj_vel_chooseparent)); % HARDFIX: default y is 1
                end
                %                 x_chooseparent = [traj_pos_chooseparent traj_vel_chooseparent];
            else % Eleva primitive
                %             traj_pos = %x(1,:);
                traj_vel_chooseparent = z_near(2)*ones(size(x_chooseparent));%x(2,:);
                traj_pos_chooseparent = z_near(1)+cumtrapz(time_chooseparent,traj_vel_chooseparent);
                traj_y_chooseparent = x_chooseparent;
            end
            x_chooseparent = [traj_pos_chooseparent(:)'; traj_vel_chooseparent(:)'; traj_y_chooseparent(:)';]; % assign arc-path
            if feasible && ~isequal(z_near(1:2),x_chooseparent(1:2,1))
                disp('WTF ChooseParent is doing?')
                keyboard
            end
        end
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
                try
                    if ((c_prime < cost_z_new) && (c_prime < c_min)) % cost_z_near) && % && (c_actual < c_x_new)
                        idx_min = idX_near(i);
                        c_min = c_prime;
                        x=x_chooseparent;
                        time = time_chooseparent;
                    end
                catch
                    keyboard
                end
            end
        end
    end
end
if ~isnan(x) & ~isequaln(z_new(1:2),x(1:2,end))
    if norm((x(1:2,end) - z_new(1:2)))/norm(z_new(1:2)) < 1e-9 % raw accuracy measure
        disp('ChooseParent is changing the goal point by a slight bit, no worries.')
    else
        disp('ChooseParent is changing the goal point by a significant amount, worry.')
        disp('If speed is constant then it''s normal')
        disp('x is')
        x
        disp('while z_new is')
        z_new
%         keyboard
    end
end
end
