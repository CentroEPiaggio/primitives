function [rewired, T, G, E , x_rewire ,pn,pe] = ReWire( idX_near, idx_min, idx_new, T, G, E, Obstacles, Ptree,idx_prim, q, cost_new,pn,pe,fig_points)
%REWIRE Summary of this function goes here
rewired = false;
traj_pos = NaN;
traj_vel = NaN;
x_rewire = NaN;
%   Detailed explanation goes here
idx_I = 1;
% make the sparse matrix square
sizeG = size(G);
[~,shorterDim]=min(sizeG);
G(sizeG(shorterDim)+1:max(sizeG),:)=0;
% get x_new
z_new = T.get(idx_new);
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
        %         keyboard
        z_near=T.get(idX_near(i));
        % begin copy from ChooseParent
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
        %         [feasible,cost_new_edge,q,traj_pos_rewire,traj_vel_rewire] = steering_muovi(X_near(1,i),x_new(1),X_near(2,i),x_new(2));
        %         z_near = z_near(:,i);
        % end copy from ChooseParent
        %         [feasible,cost_rewire,q,traj_pos_rewire,traj_vel_rewire] = steering_muovi(z_new(1),z_near(1,i),z_new(2),z_near(2,i));
        %         [feasible,cost_rewire,q,x_rewire,time_rewire] =
        %         prim.steering(z_near,z_new); % uniform interface! Yeay! % WRONG!
        %         The steering has to be done between z_new and z_near!
        [feasible,cost_rewire,q,x_rewire,time_rewire] = prim.steering(z_new,z_near); % uniform interface! Yeay!
        if feasible
            if idx_prim == 1 % collision checking only if we are on the Move primitive
                %             keyboard
                traj_pos_rewire = x_rewire(1,:);
                traj_vel_rewire = x_rewire(2,:);
                %                 keyboard
                traj_y_rewire   = z_new(3,:)*ones(size(traj_vel_rewire));
                %                 x_rewire = [traj_pos_rewire traj_vel_rewire];
            else % Eleva primitive
                %             traj_pos = %x(1,:);
                traj_vel_rewire = z_new(2)*ones(size(x_rewire));%x(2,:);
                traj_pos_rewire = z_new(1)+cumtrapz(time_rewire,traj_vel_rewire);
                traj_y_rewire = x_rewire;
                disp(['size durante la alza',num2str(size(x_rewire))]); % WHAT IS THIS?
            end
            x_rewire = [traj_pos_rewire(:)'; traj_vel_rewire(:)'; traj_y_rewire(:)';]; % assign arc-path
%             keyboard
            if isequaln(x_rewire(:,1),z_new) && isequaln(x_rewire(:,end),z_near)
                % do the rest of the rewiring, otherwise do not allow
                % rewiring
                disp('do rewire')
            else
                disp('do not rewire')
                feasible = false;
            end
            if feasible && ~isequal(z_new(1:2),x_rewire(1:2,1))
                disp('WTF ReWire is doing?')
                keyboard
            end
        end
        if feasible && ~isinf(cost_rewire) && ~isnan(cost_rewire) % last two conditions are useless, could be probably removed without problems
            if ~any(Obstacles.Node{1}.P.contains([traj_pos_rewire(:)'; traj_vel_rewire(:)'],1)) % ObstacleFree
                cost_up_to_z_new = graphshortestpath(G,idx_I,idx_new); % cost to reach z_new from the first root of the tree
                c_prime = cost_rewire;
                cost_tentative = cost_up_to_z_new + c_prime;
                cost_up_to_z_near_without_rewiring = graphshortestpath(G,idx_I,idX_near(i)); %G(idx_I,idX_near(i)); % warning: here we got the vertex-to-vertex cost only, not the total cost from the beginning to each vertex
                disp(['ReConnect costo: ' num2str(cost_tentative) ' < ' num2str(cost_up_to_z_near_without_rewiring) ' ???']);
                if cost_tentative < cost_up_to_z_near_without_rewiring && ~isinf(cost_up_to_z_near_without_rewiring) % test for correlation between sleep hours and variable length, possibly inverse proportionality
                    cprintf('[1 0.5 0]','Rewiring\n');
                    %                     keyboard
                    if checkdiscontinuity(T,E,Ptree)
                        keyboard
                    end
                    [T,G,E,pn,pe] = ReConnect(idx_new,idX_near(i),T,G,E, prim, q, cost_rewire, x_rewire, time_rewire,pn,pe,fig_points);
                    if checkdiscontinuity(T,E,Ptree)
                        keyboard
                    end
                    traj_pos = traj_pos_rewire;
                    traj_vel = traj_vel_rewire;
                    rewired = true;
                end
            end
        end
    end
end
end