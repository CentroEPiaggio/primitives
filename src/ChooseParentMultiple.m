%CHOOSEPARENT determines the best parent in the cost sense
% function idx_min = ChooseParent(idX_near, idx_nearest, T, G, x_new, cost_x_new)
function [idx_min,q,cost_new_edge,x,time,z_new,parent_found,added_intermediate_node,intermediate_primitives_list,x_list,time_list,q_list,cost_list,z_intermediate_list] ...
    = ChooseParentMultiple(idX_near, idx_nearest, T, G, E, z_new, cost_from_z_nearest_to_new,Obstacles,q,Ptree,idx_prim,idx_parent_primitive)
disp('Entered inside ChooseParent')
% make the sparse matrix square
% G = full(Graph)
% sizeG = size(G);
% [~,shorterDim]=min(sizeG);
% G(sizeG(shorterDim)+1:max(sizeG),:)=0;
disp('Inside ChooseParent')
parent_found = false;
%CHOOSEPARENT determines the best parent in the cost sense
idx_I = 1; % initial node is node 1

feasible_extend = false; % initialization

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
added_intermediate_node = false;
intermediate_primitives_list = {};
x_list = {};
q_list = {};
time_list = {};
cost_list = {};
z_intermediate_list = {};

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
        [feasible,cost_new_edge,q_chooseparent,x_chooseparent,time_chooseparent] = prim.steering(z_near,z_new); % uniform interface! Yeay!
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
                if ~all(prim.chi.P.contains([traj_pos_chooseparent(:)'; traj_vel_chooseparent(:)'],1))
                    feasible = false;
                end
            else % Eleva primitive
                %             traj_pos = %x(1,:);
                traj_vel_chooseparent = z_near(2)*ones(size(x_chooseparent));%x(2,:);
                traj_pos_chooseparent = z_near(1)+cumtrapz(time_chooseparent,traj_vel_chooseparent);
                traj_y_chooseparent = x_chooseparent;
                if ~all(prim.chi.P.contains([traj_pos_chooseparent(:)'; traj_vel_chooseparent(:)'; traj_y_chooseparent(:)'],1))
                    feasible = false;
%                     keyboard % TODO: make sure this conditions is not due to errors
                end
            end
            x_chooseparent = [traj_pos_chooseparent(:)'; traj_vel_chooseparent(:)'; traj_y_chooseparent(:)';]; % assign arc-path
            %             if isequal(prim.name,'Eleva')
            %             keyboard
            %             end
            if feasible && (~isequal(z_near(1:2),x_chooseparent(1:2,1)) || ~isequaln(round(x_chooseparent(1:length(z_new),end)*100)/100,z_new))
                disp('WTF ChooseParent is doing?')
%                 keyboard
                if isempty(idx_parent_primitive)
                    disp('WTF ChooseParent is doing? Maybe Move is doing wrong? Logging this points for further analysis.')
%                     keyboard
                    fid=fopen('log_move_wrong.txt','at');
                    fprintf(fid,'%.2f,%.2f,%.2f,%.2f\n',z_near(1), z_near(2), z_new(1), z_new(2));
                    fclose(fid);
                    feasible = false;
                    return
                end
                % BUGFIX: Fixing incongruences in the trajectory with the final point by adding intermediate trajectories made with
                % other primitives
                z_temp = round(x_chooseparent(1:length(z_new),end)*100)/100;
                dim_differences = abs(z_temp-z_new);
                map_differences = dim_differences>0;
                prim_extend = Ptree.Node{idx_parent_primitive};
                if ~isequal(prim_extend.name,prim.name)
                    if isequal(reshape(prim_extend.dimensions(1:length(map_differences)),length(map_differences),1),map_differences(:))
                        % ok, use this primitive to extend the pattern
                        [feasible_extend,cost_new_edge_extend,q_extend,x_chooseparent_extend,time_chooseparent_extend] = prim_extend.steering(z_temp,z_new); % uniform interface! Yeay!
                        if feasible_extend
                            if idx_parent_primitive == 1 % trying to fix the connection problem between different kind of primitives
                                traj_pos_chooseparent_extend = x_chooseparent_extend(1,:);
                                traj_vel_chooseparent_extend = x_chooseparent_extend(2,:);
                                if ~isnan(z_temp(3)) % HARDFIX
                                    traj_y_chooseparent_extend   = z_temp(3,:)*ones(size(traj_vel_chooseparent_extend));
                                else
                                    traj_y_chooseparent_extend   = ones(size(traj_vel_chooseparent_extend)); % HARDFIX: default y is 1
                                end
                                %                 x_chooseparent = [traj_pos_chooseparent traj_vel_chooseparent];
                                if ~all(prim_extend.chi.P.contains([traj_pos_chooseparent_extend(:)'; traj_vel_chooseparent_extend(:)'],1))
                                    feasible_extend = false;
                                end
                            else % Eleva primitive
                                %             traj_pos = %x(1,:);
                                traj_vel_chooseparent_extend = z_temp(2)*ones(size(x_chooseparent_extend));%x(2,:);
                                traj_pos_chooseparent_extend = z_temp(1)+cumtrapz(time_chooseparent_extend,traj_vel_chooseparent_extend);
                                traj_y_chooseparent_extend = x_chooseparent_extend;
                                if ~all(prim_extend.chi.P.contains([traj_pos_chooseparent_extend(:)'; traj_pos_chooseparent_extend(:)'; traj_y_chooseparent_extend(:)'],1))
                                    feasible_extend = false;
                                end
                            end
                            if feasible_extend
                                x_chooseparent_extend = [traj_pos_chooseparent_extend(:)'; traj_vel_chooseparent_extend(:)'; traj_y_chooseparent_extend(:)';]; % assign arc-path
                                
                                x_chooseparent_tentative = [x_chooseparent, x_chooseparent_extend];
                                time_chooseparent_tentative = [time_chooseparent(:)' time_chooseparent(end)+time_chooseparent_extend(:)'];
                                cost_new_edge_tentative = cost_new_edge + cost_new_edge_extend;
                                if (~isequal(z_near(1:2),x_chooseparent_tentative(1:2,1)) || ~isequaln(round(x_chooseparent_tentative(1:length(z_new),end)*100)/100,z_new))
                                    % keep_going
                                else
                                    % pack data to return from chooseparent
                                    added_intermediate_node = true;
                                    intermediate_primitives_list = {idx_prim, idx_parent_primitive};
                                    x_list = {x_chooseparent, x_chooseparent_extend};
                                    q_list = {q_chooseparent, q_extend};
                                    time_list = {time_chooseparent, time_chooseparent_extend};
                                    cost_list = {cost_new_edge, cost_new_edge_extend};
                                    z_intermediate_list = {x_chooseparent(:,end), z_new};
                                    % pack data to finish chooseparent
                                    % calculations
                                    x_chooseparent = x_chooseparent_tentative;
                                    time_chooseparent = time_chooseparent_tentative;
                                    cost_new_edge = cost_new_edge_tentative;
%                                     keyboard
                                    %                                 break;
                                end
                            end
                        end
                    end
                end
                if ~feasible_extend
                    feasible = false;
                end
            end
        end
        if feasible && ~isinf(cost_new_edge) && ~isnan(cost_new_edge) % last two conditions are useless, could be probably removed without problems
            if ~any(Obstacles.Node{1}.P.contains([traj_pos_chooseparent(:)'; traj_vel_chooseparent(:)'],1)) % ObstacleFree % TODO: obstacle avoidance here with multiple primitives
                % cost up to near vertex
                %                 cost_up_to_z_near = G(idx_I,idX_near(i));%graphshortestpath(G,idx_I,idX_near(i));
                cost_up_to_z_near = graphshortestpath(G,idx_I,idX_near(i)); % TODO this line or the one above?
                %                 cost_up_to_z_near = graphshortestpath(G,idx_I,idx_);
                % costo from near to new vertex
                cost_znear_znew  = cost_new_edge;
                %                 c_x_new = graphshortestpath(G,idx_I,idx_new);
                % costo fino al near + pezzettino near-new
                c_prime = cost_up_to_z_near + cost_znear_znew;
                
                if ((c_prime < cost_z_new) && (c_prime < c_min)) % cost_z_near) && % && (c_actual < c_x_new)
                    if feasible_extend
                        disp('Found a new trajectory by adding an intermediate node!')
%                         keyboard
                    end
                    idx_min = idX_near(i);
                    c_min = c_prime;
                    x=x_chooseparent;
                    time = time_chooseparent;
                    parent_found = true;
                else % cost not good enough
                    if feasible_extend
%                         keyboard
                        added_intermediate_node = false; % adding the nodes did not produce a lower cost so let's just connect to the nearest node.
                        
                    end
%                     feasible = false;
                end
                
            end
        end
    end
end
if ~isnan(x) & ~isequaln(z_new(1:2),round(x(1:2,end)*100/100))
    if norm((x(1:2,end) - z_new(1:2)))/norm(z_new(1:2)) < 1e-9 % raw accuracy measure
        disp('ChooseParent is changing the goal point by a slight bit, no worries.')
    else
        disp('ChooseParent is changing the goal point by a significant amount, worry.')
        disp('If speed is constant then it''s normal')
        disp('x is')
        x(:,end)
        disp('while z_new is')
        z_new
        %         keyboard
    end
end

if ~isnan(x)
    if ~isequaln(x(1:length(z_new),end),z_new)%x(1:2,end) ~= z_new(1:2)
        disp('ChooseParent slightly changed the goal point!')
        x(1:length(z_new),end)
        z_new
        %         keyboard
        %                         if pushed_in_goal
        %                             reached(x(1:length(z_new),end),z_new)
        %                             keyboard
        %                         end
        % this should fix the discontinuity problem
        %                                [z_new,x] = truncate_to_similar(z_new,x);
    else
        disp('ChooseParent was actually good!')
        keyboard
    end
end

% is this really needed now?
if any(any(~isnan(x_chooseparent)))
    %     disp('fix this')
    %     keyboard
    z_new = round(x_chooseparent(prim.dimensions>0,end)*100)/100; % HACK to ensure continuity in the trajectories stored in the tree
end

end
