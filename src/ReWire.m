function [T, G, E , traj_pos,traj_vel,pn,pe] = ReWire( idX_near, idx_min, idx_new, T, G, E, Obstacles, prim, q, cost_new,pn,pe,fig_points)
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
    % CHECK EVERYTIME THE DIMENSIONS OF THE GRAPH TO BE CONSISTENT BTW THEM
    sizeG = size(G);
    [~,shorterDim]=min(sizeG);
    G(sizeG(shorterDim)+1:max(sizeG),:)=0;

    if (idX_near(i)==idx_min || idX_near(i)==idx_I) % avoid idx_min and tree root
        continue;
    end
    if ~isempty(idX_near(i))
        X_near(:,i)=T.get(idX_near(i));
        [feasible,cost_rewire,q,traj_pos_rewire,traj_vel_rewire] = steering_muovi(x_new(1),X_near(1,i),x_new(2),X_near(2,i));
        if feasible && ~isinf(cost_rewire) && ~isnan(cost_rewire) % last two conditions are useless, could be probably removed without problems
            if ~any(Obstacles.Node{1}.P.contains([traj_pos_rewire(:)'; traj_vel_rewire(:)'],1)) % ObstacleFree
                cost_up_to_z_new = graphshortestpath(G,idx_I,idx_new); % cost to reach z_new from the first root of the tree
                c_prime = cost_rewire;
                cost_tentative = cost_up_to_z_new + c_prime;
                cost_up_to_z_near_without_rewiring = graphshortestpath(G,idx_I,idX_near(i)); %G(idx_I,idX_near(i)); % warning: here we got the vertex-to-vertex cost only, not the total cost from the beginning to each vertex
                disp(['ReConnect costo: ' num2str(cost_tentative) ' < ' num2str(cost_up_to_z_near_without_rewiring) ' ???']);
                if cost_tentative < cost_up_to_z_near_without_rewiring && ~isinf(cost_up_to_z_near_without_rewiring) % test for correlation between sleep hours and variable length, possibly inverse proportionality
                    cprintf('[1 0.5 0]','Rewiring\n');
                    keyboard
                    [T,G,E,pn,pe] = ReConnect(idx_new,idX_near(i),T,G,E, prim, q, cost_rewire,pn,pe,fig_points);
                    traj_pos = traj_pos_rewire;
                    traj_vel = traj_vel_rewire;
                end
            end
        end
    end
end
end