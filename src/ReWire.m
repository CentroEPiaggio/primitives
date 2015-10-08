function [ T, G, E , traj_pos,traj_vel, h_traj_vector] = ReWire( idX_near, idx_min, idx_new, T, G, E, Obstacles, prim, q, cost_new,verbose,cerchio,debug,h_traj_vector)
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
% if debug, keyboard, end
if verbose
    disp('Inside Rewire');
    %     if debug, keyboard, end
    set(cerchio,'visible','on')
    %     if debug, keyboard, end
    h_idx_near = zeros(size(idX_near));
    for i=1:length(idX_near)
        if idX_near(i) == 1 % no need to rewire the first node
            continue;
        end
        x_i = T.get(idX_near(i));
        h_idx_near(i) = plot(x_i(1),x_i(2),'ro','linewidth',2);
    end
end
h_traj_rewire = zeros(size(idX_near));
h_traj_reconnect = 0;
for i=1:length(idX_near) % for every point btw the nearby vertices
    % CHECK EVERYTIME THE DIMENSIONS OF THE GRAPH TO BE CONSISTENT BTW THEM
    sizeG = size(G);
    [~,shorterDim]=min(sizeG);
    G(sizeG(shorterDim)+1:max(sizeG),:)=0;
    
    if idX_near(i)==idx_min || idX_near(i)==idx_new % avoid idx_min and idx_new
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
        if verbose
            h_traj_rewire(i)=plot(traj_pos_rewire,traj_vel_rewire,'r','linewidth',2);
            if debug, keyboard, end
        end
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
                    cprintf('[1 0 1]','ReConnect\n');
                    keyboard
                    %                     avviso
                    %                     if debug, keyboard, end
                    [T,G,E] = ReConnect(idx_new,idX_near(i),T,G,E, prim, q, cost_rewire);
                    traj_pos = traj_pos_rewire;
                    traj_vel = traj_vel_rewire;
                    if verbose
                        delete(h_traj_vector(idX_near(i))); % remove old edge in the plot
                        h_traj_reconnect(i)=plot(traj_pos,traj_vel,'b','linewidth',2); % draw reconnect edge in yellow
                        h_traj_vector(idX_near(i))=h_traj_reconnect(i);
                    end
                end
            end
        end
    end
end
if verbose
    set(cerchio,'visible','off')
    if debug, keyboard, end
    delete(h_idx_near(h_idx_near>0));
    delete(h_traj_rewire(h_traj_rewire>0));
    delete(h_traj_reconnect(h_traj_reconnect>0));
end
end