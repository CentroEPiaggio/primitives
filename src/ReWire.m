function [ T, G, E , traj_pos,traj_vel] = ReWire( idX_near, idx_min, idx_new, T, G, E, Obstacles, prim, q, cost_new)
%REWIRE Summary of this function goes here
traj_pos = NaN;
traj_vel = NaN;
%   Detailed explanation goes here
idx_I = 1;
% make the sparse matrix square
sizeG = size(G);
[~,shorterDim]=min(sizeG);
G(sizeG(shorterDim)+1:max(sizeG),:)=0;
% get x_new
x_new = T.get(idx_new);
% keyboard
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
        [feasible,cost_rewire,q,traj_pos_rewire,traj_vel_rewire] = steering_muovi(x_new(1),X_near(1,i),x_new(2),X_near(2,i));
        disp('Sono davvero dentro rewire');
        if feasible && ~isinf(cost_rewire) && ~isnan(cost_rewire) % last two conditions are useless, could be probably removed without problems
            disp('Sono davvero dentro rewire e feasible');
            if ~any(Obstacles.Node{1}.P.contains([traj_pos_rewire(:)'; traj_vel_rewire(:)'])) % ObstacleFree
                disp('Sono davvero dentro rewire e feasible e obstaclefree');
                %cost_up_to_z_near = graphshortestpath(G,idx_I,idX_near(i)); % TODO: well, this should be taken from the cost graph
%cost_up_to_z_new = cost_new;
                cost_up_to_z_new = graphshortestpath(G,idx_I,idx_new); % cost to reach z_new from the first root of the tree
                c_prime = cost_rewire;
                cost_tentative = cost_up_to_z_new + c_prime;
                cost_up_to_z_near_without_rewiring = graphshortestpath(G,idx_I,idX_near(i)); %G(idx_I,idX_near(i)); % warning: here we got the vertex-to-vertex cost only, not the total cost from the beginning to each vertex
                disp(['ReConnect costo: ' num2str(cost_tentative) ' < ' num2str(cost_up_to_z_near_without_rewiring) ' ???']);
                if cost_tentative < cost_up_to_z_near_without_rewiring && ~isinf(cost_up_to_z_near_without_rewiring) % test for correlation between sleep hours and variable length, possibly inverse proportionality
                    disp('OK, once here thou shalt rewire')
                    disp('ReConnect');
                    avviso
                    keyboard
                    [T,G,E] = ReConnect(idx_new,idX_near(i),T,G,E, prim, q, cost_new);
                    traj_pos = traj_pos_rewire;
                    traj_vel = traj_vel_rewire;
                end
            end
        end
    end
end
end