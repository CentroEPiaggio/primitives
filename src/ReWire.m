function [rewired, T, G, E , x_rewire ,pn,pe, added_new, idx_last_added] = ReWire( idX_near, idx_min, idx_new, z_new, T, G, E, Obstacles, Ptree,idx_prim, q, cost_new,pn,pe,fig_points,verbose)
% keyboard
%REWIRE Summary of this function goes here
% keyboard
rewired = false;
added_new = false;
idx_last_added = idx_new;
% traj_pos = NaN;
% traj_vel = NaN;
x_rewire = NaN;
%   Detailed explanation goes here
idx_I = 1;
% get x_new
% z_new = T.get(idx_new);
% keyboard
for i=1:length(idX_near) % for every point btw the nearby vertices
    if (idX_near(i)==idx_min || idX_near(i)==idx_I) % avoid idx_min and tree root
        continue;
    end
    if ~isempty(idX_near(i))
        %         keyboard
        z_near=T.get(idX_near(i));
        % begin copy from ChooseParent
        % select primitive HARDFIX
        % BUGFIX (*)
        if ~isequal(isnan(z_near),isnan(z_new))
            continue
            %             return
        end
        % finds the primitive space where the points both live
        prim_found = false;
        for jj=1:Ptree.nnodes
            prim = Ptree.Node{jj};
            if all(prim.chi.P.contains([z_new(prim.dimensions_imagespace>0),z_near(prim.dimensions_imagespace>0)])) && isequaln(z_new(prim.dimensions_imagespace==0),z_near(prim.dimensions_imagespace==0))
                prim_found = true;
                break;
            end
        end
        if ~prim_found
            disp('Could not find a primitive for rewiring!');
            keyboard
        end
        % calculate feasibility and cost
        %         [feasible,cost_new_edge,q,traj_pos_rewire,traj_vel_rewire] = steering_muovi(X_near(1,i),x_new(1),X_near(2,i),x_new(2));
        %         z_near = z_near(:,i);
        % end copy from ChooseParent
        %         [feasible,cost_rewire,q,traj_pos_rewire,traj_vel_rewire] = steering_muovi(z_new(1),z_near(1,i),z_new(2),z_near(2,i));
        %         [feasible,cost_rewire,q,x_rewire,time_rewire] =
        %         prim.steering(z_near,z_new); % uniform interface! Yeay! % WRONG!
        %         The steering has to be done between z_new and z_near!
        
        cost_up_to_z_near_without_rewiring = graphshortestpath(G,idx_I,idX_near(i)); %G(idx_I,idX_near(i)); % warning: here we got the vertex-to-vertex cost only, not the total cost from the beginning to each vertex
        cost_up_to_z_new = graphshortestpath(G,idx_I,idx_new); % cost to reach z_new from the first root of the tree
        
        if cost_up_to_z_near_without_rewiring < cost_up_to_z_new % in this condition it is impossible to achieve a lower cost passing through z_new to get to z_near
            %             keyboard
            continue;
        end
        
        %keyboard
        try
            [feasible,cost_rewire,q,x_rewire,time_rewire] = prim.steering(z_new,z_near); % uniform interface! Yeay!
        catch ERR
            disp(ERR.message)
            keyboard
        end
        if feasible
            %keyboard
            try
                [x_rewire] = complete_trajectories(z_new,time_rewire,x_rewire,Ptree,prim.ID);
                if ~isequaln(round(x_rewire(prim.dimensions>0,end)*100)/100,z_near(prim.dimensions>0))
                    feasible = false;
                end
            catch ERR
                disp(ERR.message)
                keyboard
            end
        end
        if feasible
            feasible=CollisionFree(x_rewire,Ptree,Obstacles); % Here
        end
        if feasible
%             keyboard
            %             if idx_prim == 1 % collision checking only if we are on the Move primitive
            %                 %             keyboard
            %                 traj_pos_rewire = x_rewire(1,:);
            %                 traj_vel_rewire = x_rewire(2,:);
            %                 %                 keyboard
            %                 traj_y_rewire   = z_new(3,:)*ones(size(traj_vel_rewire)); % BUG: this line generates NaN trajectories when z_new(3,:) is NaN! FIXED IN BUGFIX (*)
            %                 %                 x_rewire = [traj_pos_rewire traj_vel_rewire];
            %             else % Eleva primitive
            %                 %             traj_pos = %x(1,:);
            %                 traj_vel_rewire = z_new(2)*ones(size(x_rewire));%x(2,:);
            %                 traj_pos_rewire = z_new(1)+cumtrapz(time_rewire,traj_vel_rewire);
            %                 traj_y_rewire = x_rewire;
            %             end
            %             x_rewire = [traj_pos_rewire(:)'; traj_vel_rewire(:)'; traj_y_rewire(:)';]; % assign arc-path
            %             keyboard
            %             if isequaln(x_rewire(:,1),z_new) && isequaln(x_rewire(:,end),z_near)
            %% here we try to add an intermediate node in case that some trimmered primitive has changed some dimensions of the goal point
            if idx_prim>1
                %                 keyboard
                %                 figure,plot(time_rewire,x_rewire,time_rewire(1)*ones(3,1),z_new,'ro',time_rewire(end)*ones(3,1),z_near,'bo')
            end
            try
                %keyboard
                idx_parent_primitive = 1; % HARDCODED
                [added_intermediate_node,intermediate_primitives_list,x_list,time_list,cost_list,q_list,z_list] = intermediate_node(time_rewire,x_rewire,z_new,z_near,prim,Ptree,idx_parent_primitive,Obstacles);
            catch MINTPRIM
                disp(MINTPRIM.message)
                %keyboard
            end
            %%
            if added_intermediate_node || ( isequaln(round(x_rewire(prim.dimensions>0,1)*100)/100,round(z_new(prim.dimensions>0)*100)/100) && isequaln(round(x_rewire(prim.dimensions>0,end)*100)/100,z_near(prim.dimensions>0)))
                % do the rest of the rewiring, otherwise do not allow
                % rewiring
                %                 disp('do rewire')
                cprintf('*[1 0.5 0]*','do rewire if cost is convenient\n');
            else
                %                 disp('do not rewire')
                cprintf('*[1 0.5 0]*','do not rewire\n');
                feasible = false;
            end
            if feasible && ~isequal(z_new(1:2),x_rewire(1:2,1))
                disp('WTF ReWire is doing?')
                keyboard
            end
        end
        %         feasible=false; %WARNING!! FORCING NOT TO DO REWIRE!
        %% Cost checking for ReWire
        if feasible && ~isinf(cost_rewire) && ~isnan(cost_rewire) % last two conditions are useless, could be probably removed without problems
            if ~any(Obstacles.Node{1}.P.contains([traj_pos_rewire(:)'; traj_vel_rewire(:)'],1)) % TODO: ObstacleFree
                
                if added_intermediate_node
                    c_prime = sum([cost_list{:}]); % sum cost of passing between all intermediate nodes
                else
                    c_prime = cost_rewire;
                    intermediate_primitives_list = {prim.ID};
                    x_list = {x_rewire};
                    time_list = {time_rewire};
                    cost_list = {cost_rewire};
                    q_list = {q};
                end
                cost_tentative = cost_up_to_z_new + c_prime;
                
                cprintf('*[1 0.5 0]*','ReConnect cost: %f < %f ??? ',cost_tentative,cost_up_to_z_near_without_rewiring);
                if cost_tentative < cost_up_to_z_near_without_rewiring
                    cprintf('*[0 1 0]*','YES!!!\n');
                    %                     keyboard
                else
                    cprintf('*[1 0 0]*','NO!!!\n');
                end
                if cost_tentative < cost_up_to_z_near_without_rewiring && ~isinf(cost_up_to_z_near_without_rewiring) % test for correlation between sleep hours and variable length, possibly inverse proportionality
                    cprintf('*[1 0.5 0]*','Rewiring\n');
                    %                     keyboard
                    if checkdiscontinuity(T,E,Ptree)
                        keyboard
                    end
                    %                     keyboard
                    if added_intermediate_node
                        cprintf('*[1 0.5 0]*','ReConnect node through intermediate nodes\n');
                        if any(any(isnan([x_list{:}])))
                            disp('NaNs!!!!')
                            keyboard
                        end
                    else
                        cprintf('*[1 0.5 0]*','ReConnect node: %d to pass through node %d instead of node %d\n',idX_near(i),idx_new,T.Parent(idX_near(i)));
                        if any(any(isnan(x_rewire)))
                            disp('NaNs!!!!')
                            keyboard
                        end
                    end
                    %                     tmp=line([z_new(1) z_near(1)],[z_new(2) z_near(2)],'color','yellow','linewidth',2,'linestyle','-.');
                    %                     oldparents = T.Parent;
                    
                    [T,G,E,pn,pe,added_new] = ReConnect(idx_new,idX_near(i),T,G,E,Ptree, intermediate_primitives_list, q_list, cost_list, x_list, time_list, z_list, pn,pe,fig_points,verbose);
                    %                     [oldparents,T.Parent]
                    %                     keyboard
                    %                     delete(tmp);
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