function [rewired, T, G, E , x_rewire ,pn,pe] = ReWire( idX_near, idx_min, idx_new, T, G, E, Obstacles, Ptree,idx_prim, q, cost_new,pn,pe,fig_points)
% keyboard
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
            return
        end
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
        
        cost_up_to_z_near_without_rewiring = graphshortestpath(G,idx_I,idX_near(i)); %G(idx_I,idX_near(i)); % warning: here we got the vertex-to-vertex cost only, not the total cost from the beginning to each vertex
        cost_up_to_z_new = graphshortestpath(G,idx_I,idx_new); % cost to reach z_new from the first root of the tree
        
        if cost_up_to_z_near_without_rewiring < cost_up_to_z_new % in this condition it is impossible to achieve a lower cost passing through z_new to get to z_near
            %             keyboard
            continue;
        end
        
        [feasible,cost_rewire,q,x_rewire,time_rewire] = prim.steering(z_new,z_near); % uniform interface! Yeay!
        if feasible
            if idx_prim == 1 % collision checking only if we are on the Move primitive
                %             keyboard
                traj_pos_rewire = x_rewire(1,:);
                traj_vel_rewire = x_rewire(2,:);
                %                 keyboard
                traj_y_rewire   = z_new(3,:)*ones(size(traj_vel_rewire)); % BUG: this line generates NaN trajectories when z_new(3,:) is NaN! FIXED IN BUGFIX (*)
                %                 x_rewire = [traj_pos_rewire traj_vel_rewire];
            else % Eleva primitive
                %             traj_pos = %x(1,:);
                traj_vel_rewire = z_new(2)*ones(size(x_rewire));%x(2,:);
                traj_pos_rewire = z_new(1)+cumtrapz(time_rewire,traj_vel_rewire);
                traj_y_rewire = x_rewire;
            end
            x_rewire = [traj_pos_rewire(:)'; traj_vel_rewire(:)'; traj_y_rewire(:)';]; % assign arc-path
            %             keyboard
            %             if isequaln(x_rewire(:,1),z_new) && isequaln(x_rewire(:,end),z_near)
            if isequaln(round(x_rewire(prim.dimensions>0,1)*100)/100,round(z_new(prim.dimensions>0)*100)/100) && isequaln(round(x_rewire(prim.dimensions>0,end)*100)/100,z_near(prim.dimensions>0))
                % do the rest of the rewiring, otherwise do not allow
                % rewiring
                %                 disp('do rewire')
                cprintf('*[1 0.5 0]*','do rewire\n');
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
        if feasible && ~isinf(cost_rewire) && ~isnan(cost_rewire) % last two conditions are useless, could be probably removed without problems
            if ~any(Obstacles.Node{1}.P.contains([traj_pos_rewire(:)'; traj_vel_rewire(:)'],1)) % ObstacleFree
                
                c_prime = cost_rewire;
                cost_tentative = cost_up_to_z_new + c_prime;
                
                disp(['ReConnect costo: ' num2str(cost_tentative) ' < ' num2str(cost_up_to_z_near_without_rewiring) ' ???']);
                cprintf('*[1 0.5 0]*','ReConnect cost: %f < %f ??? ',cost_tentative,cost_up_to_z_near_without_rewiring);
                if cost_tentative < cost_up_to_z_near_without_rewiring
                    cprintf('*[0 1 0]*','YES!!!\n');
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
                    cprintf('*[1 0.5 0]*','ReConnect node: %d to pass through node %d instead of node %d\n',idX_near(i),idx_new,T.Parent(idX_near(i)));
                    %                     tmp=line([z_new(1) z_near(1)],[z_new(2) z_near(2)],'color','yellow','linewidth',2,'linestyle','-.');
                    oldparents = T.Parent;
                    if any(any(isnan(x_rewire)))
                        disp('NaNs!!!!')
                        keyboard
                    end
                    [T,G,E,pn,pe] = ReConnect(idx_new,idX_near(i),T,G,E, prim, q, cost_rewire, x_rewire, time_rewire,pn,pe,fig_points);
                    [oldparents,T.Parent]
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