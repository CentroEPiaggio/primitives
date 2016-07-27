function [Tree,G,E,z_new,plot_nodes,plot_edges,feasible,added_new,idx_last_added] = localRRTstar(Chi,Ptree,idx_prim,z_rand,T,Graph,Edges,Obstacles,verbose,plot_nodes,plot_edges,pushed_in_goal,goal_node,idx_parent_primitive,gam,tol,replicate_over_primitive)
debug = 1;

cprintf('*[0,0.7,1]*','# entering localRRTstar #\n');
% keyboard
% select current primitive
prim = Ptree.get(idx_prim);

if length(replicate_over_primitive)>0
    replicate_with = {fix_nans(z_rand,prim.dimensions_imagespace)};
    for cc=1:length(replicate_over_primitive)
        if replicate_over_primitive~=prim.ID
            prim_replicate = Ptree.Node{replicate_over_primitive(cc)};
            %     replicate_with = {cell2mat(replicate_with), prim_replicate.replicate(z_rand)}; % TODO: generic implementation
            replicate_with = {cell2mat(replicate_with), [z_rand;0], [z_rand;1]}; % TODO: HARDCODED
%             replicate_with = {[z_rand;1]}; % TODO: HARDCODED
            %-keyboard
        end
    end
else
    %-keyboard
    replicate_with = {fix_nans(z_rand,prim.dimensions_imagespace)};
end
for bb=1:length(replicate_with)
%     if bb==1 && pushed_in_goal
%         disp('Let''s fix this motherfucker');
%         keyboard
%     end
    %%
    fig_xv=2; fig_xy = 3; fig_yv = 4; % stuff to plot
    figure(fig_xy);
    plot3(replicate_with{bb}(1),replicate_with{bb}(2),replicate_with{bb}(5),'rx','linewidth',2)
    
    % initialization values
    feasible = false;
    rewired = false;
    added_new = false;
    idx_last_added = NaN;
    
    z_new=z_rand;
    
    added_intermediate_node = false;
    
    if pushed_in_goal
        disp(['pushed_in_goal is true: z_rand is ' num2str(z_rand(:)')]);
        %     %-keyboard
    end
    
    
    
    cprintf('*[0,0.7,1]*','* looking for nearest sample *\n');
    % find nearest point in the tree
    if ~isempty(idx_parent_primitive)
        parent_prim = Ptree.Node{idx_parent_primitive};
    else
        parent_prim = [];
    end
    %     [idx_nearest,z_nearest] = Nearest(z_rand,T,prim,parent_prim);
    [idx_nearest,z_nearest] = Nearest(replicate_with{bb},T,prim,parent_prim);
    % make sure that we are not attaching to the goal_node...
    if ~isempty(goal_node)
        idx_nearest(idx_nearest==goal_node) = [];
    end
    if isempty(idx_nearest) % ... if the goal noad is the only nearest node, then go away
        % do nothing
        Tree = T;
        G = Graph;
        E = Edges;
        disp('Do nothing inside localRRTstar');
        %     %-keyboard
        %         return
        continue
    end
    % rescale z_rand within a ball of radius \eta centered in z_nearest
    cprintf('*[0,0.7,1]*','* rescaling z_rand close to nearest sample *\n');
    eta = {1.2;0.1}; % TUNABLE PARAMETER. TODO: replace this with prim.eta inside the primitive's initialization
    if idx_prim>1
        %     error('remember to fix this eta radius bubble stuff!');
        cprintf('*[0,0.7,1]*','* remember to fix this eta radius bubble stuff! *\n');
    end
    % %-keyboard
    if prim.check_extendable(z_nearest)
        z_nearest_extended = prim.extend(z_nearest);
    else
        z_nearest_extended = z_nearest;
    end
    try
        if norm(z_rand-z_nearest_extended(prim.dimensions_imagespace>0)) > eta{prim.ID}
            alfa = eta{prim.ID}/norm(z_rand-z_nearest_extended(prim.dimensions_imagespace>0)); % TODO check dimensions over other primitives!
            z_rand = (1-alfa)*z_nearest_extended(prim.dimensions_imagespace>0)+(alfa)*z_rand;
            z_rand = round(z_rand*100)/100;
            z_new = z_rand;
        end
    catch ME
        disp(ME.message);
        %-keyboard
    end
    % check if both points are in the image space of the primitive. This should
    % be a redundant check and could be removed later on.
    if all( prim.chi.P.contains([z_rand, z_nearest_extended(prim.dimensions_imagespace>0)],1) )
        cprintf('*[0,0.7,1]*','* Steering between nearest and random sample *\n');
        [feasible,cost,q,x,time] = prim.steering(z_nearest,z_rand); % uniform interface! Yeay!
        dim_z_new = prim.dimensions_imagespace;
        if prim.ID == 2
            %keyboard
        end
        %         if prim.ID == 1 && z_nearest_extended(5)==1
        %             keyboard
        %         end
        if feasible
            
            % collision checking
            cprintf('*[0,0.7,1]*','* Collision detection between nearest and random sample *\n');
            disp(['before collisionfree with primitive ' prim.name])
            try
                [x_complete] = complete_trajectories(z_nearest,time,x,Ptree,prim.ID);
            catch ME
                disp(ME.message)
                keyboard
            end
            %         x = x_complete;
            %         z_new(prim.dimensions==0) = x(prim.dimensions==0,end);
            %         feasible=CollisionFree(x,Ptree,Obstacles);
            try
                feasible = CollisionFree(x_complete,Ptree,Obstacles);
            catch ME
                disp(ME.message)
                keyboard
            end
            disp(['after  collisionfree with primitive ' prim.name])
            if checkdiscontinuity(T,Edges,Ptree)
                %-keyboard
            end
        end
        if checkdiscontinuity(T,Edges,Ptree)
            %-keyboard
        end
        %     if feasible && ~isequal(z_nearest(1:2),x(1:2,1))
        %         disp('WTF?')
        %         %-keyboard
        %     end
        if feasible
            cprintf('*[0,0.7,1]*','* Search for Near samples in the tree *\n');
            
            cardV = T.nnodes; % number of vertices in the graph
            try
                %                 keyboard
                z_new_near = replicate_with{bb};
                [idx_near_bubble,raggio] = near(T,Graph,Edges,z_new_near,dim_z_new,cardV,gam,prim);     % Check for nearest point inside a certain bubble
                
                %                 [idx_near_bubble,raggio] = near(T,Graph,Edges,z_new,dim_z_new,cardV,gam);     % Check for nearest point inside a certain bubble
            catch ME
                disp(ME.message);
                %-keyboard
            end
            if ~isempty(goal_node)
                idx_near_bubble(idx_near_bubble==goal_node) = [];
            end
            idx_near_bubble
            disp(['### begin radius loop: raggio=' num2str(raggio)])
            if isinf(raggio)
                disp('infinite radius, why?');
                %-keyboard
            end
            if raggio>0 %&& ~isempty(idx_near_bubble)%&& ~isempty(idx_near_bubble) %
                centro = z_new(1:2)-raggio;
                diameter = 2*raggio;
                figure(fig_xv)
                cerchio = rectangle('position',[centro',diameter,diameter],... % Draw a circle around the nearest neighbors inside the bubble.
                    'curvature',[1 1],'EdgeColor','b'); % 'LineStyle',':'
                if verbose
                    set(cerchio,'visible','on')
                end
                
                % Find the parent with the lower cost
                cprintf('*[0,0.7,1]*','* ChooseParent with the lower cost in the neighborhood *\n');
                cost_from_z_nearest_to_new = cost;
                if isempty(idx_near_bubble) || length(idx_near_bubble) == 1                                 % if there is no near vertex in the bubble (or there is just the nearest) keep the nearest node and proceed to insert it in the tree
                    cprintf('*[0,0.7,1]*','* Near set is empty, attempting to connect to (eventually scaled) nearest point *\n');
                    idx_min = idx_nearest;
                    cost_new = cost_from_z_nearest_to_new;
                    [x_complete] = complete_trajectories(T.get(idx_min),time,x,Ptree,prim.ID);
                    x = x_complete;
                else                                                        % otherwise look for possibly more convenient paths
                    %                     if bb==3
                    %                         keyboard
                    %                     end
                    z_new_chooseparent = replicate_with{bb};
                    try
                        [idx_min,q,cost_new,x_chooseparent,time_chooseparent,z_new_chooseparent,...
                            parent_found,added_intermediate_node,intermediate_primitives_list,x_list,time_list,...
                            q_list,cost_list,z_intermediate_list,feasible_chooseparent] = ChooseParentMultiple(idx_near_bubble, idx_nearest, T, Graph, Edges, z_new_chooseparent,cost_from_z_nearest_to_new,Obstacles,q,Ptree,idx_prim,idx_parent_primitive);
                    catch ME
                        disp(ME.message)
                        keyboard
                    end
                    %                 if ~parent_found
                    if ~parent_found
                        idx_min = idx_nearest;
                        cost_new = cost_from_z_nearest_to_new;
                        [x_complete] = complete_trajectories(T.get(idx_min),time,x,Ptree,prim.ID);
                        x = x_complete;
                    else
                        try
                            [x_complete] = complete_trajectories(T.get(idx_min),time_chooseparent,x_chooseparent,Ptree,prim.ID);
                        catch ME
                            disp(ME.message)
                            keyboard
                        end
                        x = x_complete;
                        z_new = round(z_new_chooseparent*100)/100;
                    end
                    if isinf(cost_new) % no feasible neighbor found
                        feasible=false;
                        cprintf('*[1,0.5,1]*','* !!!!!!!!!!!!!!!!!! ChooseParent has not found any viable parent and nearest is not reachable. !!!!!!!!!!!!!!!!!! *\n');
                        if debug
                            %-keyboard
                        end
                    end
                    %                 %-keyboard % TODO INSERT trim_trajectory
                    if feasible && ~any(any(isnan(x)))
                        %                     [x_complete] = complete_trajectories(T.get(idx_min),time_chooseparent,x_chooseparent,Ptree,prim.ID);
                        %                     x = x_complete;
                        %                     traj_pos = x_chooseparent(1,:);
                        %                     traj_vel = x_chooseparent(2,:);
                        %                     traj_y = x_chooseparent(3,:); % TODO: FIX NAMES
                        %                     x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)']; % assign arc-path
                        %                     z_new_temp = z_new;
                        %                     z_new = round(z_new*100)/100;
                        if ~isequaln(round(x(1:length(z_new),end)*100)/100,z_new)%x(1:2,end) ~= z_new(1:2)
                            disp('ChooseParent slightly changed the goal point!')
                            %                         %-keyboard
                            %                             if pushed_in_goal
                            %                                 if reached(x(:,end),z_new,tol)
                            %                                     %-keyboard
                            %                                 end
                            %                             end
                        end
                    end
                end
                if checkdiscontinuity(T,Edges,Ptree)
                    %-keyboard
                end
                if feasible
                    cprintf('*[0,0.7,1]*','* Proceed to InsertNode *\n');
                    if ~added_intermediate_node
                        %                     if idx_prim==1
                        % z_start = x(:,1);z_end = x(:,end); figure,plot(time,x,time(1)*ones(4,1),z_start,'ro',time(end)*ones(4,1),z_end,'ro',time,prim.chi.P.contains(x(prim.dimensions>0,:))),grid on,legend('x','y','th','v','z\_start','z\_end','contains check','location','best')
                        if prim.ID == 2
                            xplottraj = x(prim.dimensions_imagespace>0,:);
                            figure(33),clf;prim.chi.P.plot('color','lightblue','alpha',0.5); hold on; plot3(xplottraj(1,:),xplottraj(2,:),xplottraj(3,:))
                            idx_inside_primitive = find(prim.chi.P.contains(x(prim.dimensions_imagespace>0,:)));
                            cost = cost*time(idx_inside_primitive(end))/time(end);
                            time = time(idx_inside_primitive);
                            x = x(:,idx_inside_primitive);
                            xplottraj = x(prim.dimensions_imagespace>0,:);
                            plot3(xplottraj(1,:),xplottraj(2,:),xplottraj(3,:))
                            %-keyboard
                        end
                        if all(prim.chi.P.contains(x(prim.dimensions_imagespace>0,:)))
                            [x_complete] = complete_trajectories(T.get(idx_min),time,x,Ptree,prim.ID);
                            x = x_complete;
                            z_new = round(x(prim.dimensions_imagespace>0,end)*100)/100; % TO FIX DISCONTINUITY PROBLEM
                            if isempty(idx_min)
                                %-keyboard
                            end
                            [added_new,T,Graph,Edges,plot_nodes,plot_edges,idx_last_added] = InsertNode(idx_min, z_new, T, Graph, Edges, prim, q, cost_new, x, time, verbose, plot_nodes, plot_edges);
                        else
                            disp('Trajectory outside admissible space.')
                        end
                        %                     else % idx_prim > 1
                        %                         if all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'; traj_y(:)'],1))
                        %                             z_new = round(x(prim.dimensions>0,end)*100)/100; % TO FIX DISCONTINUITY PROBLEM
                        %                             if isempty(idx_min)
                        %                                 %-keyboard
                        %                             end
                        %                             [added_new,T,Graph,Edges,plot_nodes,plot_edges,idx_last_added] = InsertNode(idx_min, z_new, T, Graph, Edges, prim, q, cost_new, x, time, verbose, plot_nodes, plot_edges);
                        %                         else
                        %                             disp('Trajectory outside admissible space.')
                        %                         end
                        %                     end
                    else
                        %                     %-keyboard
                        for kk=1:length(intermediate_primitives_list)
                            %%
                            prim_intermediate = Ptree.get(intermediate_primitives_list{kk});
                            if intermediate_primitives_list{kk}==1
                                traj_pos = x_list{kk}(1,:);
                                traj_vel = x_list{kk}(2,:);
                                if all(prim_intermediate.chi.P.contains([traj_pos(:)'; traj_vel(:)'],1)) % TODO: MISSING COLLISION AVOIDANCE HERE???
                                    z_new_intermediate = round(x_list{kk}(prim.dimensions>0,end)*100)/100; % TO FIX DISCONTINUITY PROBLEM
                                    if isempty(idx_min)
                                        %-keyboard
                                    end
                                    [added_new,T,Graph,Edges,plot_nodes,plot_edges,idx_last_added] = InsertNode(idx_min, z_new_intermediate, T, Graph, Edges, prim_intermediate, q_list{kk}, cost_list{kk}, x_list{kk}, time_list{kk}, verbose, plot_nodes, plot_edges);
                                else
                                    disp('Trajectory outside admissible space.')
                                end
                            else % idx_prim > 1
                                %                             if all(prim_intermediate.chi.P.contains([traj_pos(:)'; traj_vel(:)'; traj_y(:)'],1))  % TODO: MISSING COLLISION AVOIDANCE HERE???
                                if all(prim_intermediate.chi.P.contains([x_list{kk}],1)) % TODO: MISSING COLLISION AVOIDANCE HERE???
                                    z_new_intermediate = round(x_list{kk}(prim_intermediate.dimensions>0,end)*100)/100; % TO FIX DISCONTINUITY PROBLEM
                                    if isempty(idx_min)
                                        %-keyboard
                                    end
                                    [added_new,T,Graph,Edges,plot_nodes,plot_edges,idx_last_added] = InsertNode(idx_min, z_new_intermediate, T, Graph, Edges, prim_intermediate, q_list{kk}, cost_list{kk}, x_list{kk}, time_list{kk}, verbose, plot_nodes, plot_edges);
                                else
                                    disp('Trajectory outside admissible space.')
                                end
                            end
                            %                         [added_new,T,Graph,Edges] = InsertNode(idx_min, z_intermediate_list{kk}, T, Graph, Edges, prim_intermediate, q_list{kk}, cost_list{kk}, x_list{kk}, time_list{kk});
                            if added_new
                                idx_min = T.nnodes; % TODO: warning, this should point to the parent of the last added node but it might point directly to the last node in this case
                            end
                            %%
                            if checkdiscontinuity(T,Edges,Ptree)
                                %-keyboard
                            end
                        end
                    end
                    dimG = size(Graph);
                    if dimG(1) ~= dimG(2)
                        %-keyboard;
                    end
                    if added_new
                        cprintf('*[0,1,0]*','! Node Added !\n');
                    else
                        cprintf('*[1,0,0]*','! Node Not Added !\n');
                    end
                    
                    if checkdiscontinuity(T,Edges,Ptree)
                        %-keyboard
                    end
                    
                    cprintf('*[0,0.7,1]*','* WARNING: PREVENTING REWIRE! *\n'); % WARNING: PREVENTING REWIRE!
                    rewire_enable = true;
                    try
                        if added_new && T.nnodes>2 && ~isempty(idx_near_bubble) && rewire_enable
                            cprintf('*[0,0.7,1]*','* ReWire *\n');
                            %                     z_min = T.get(idx_min);
                            idx_new = T.nnodes;
                            %                     if ReWire(idx_near_bubble, idx_min, idx_new, T, Graph, Edges, Obstacles, Ptree,idx_prim, q, cost_new,plot_nodes,plot_edges,fig_xv);
                            %                         %-keyboard
                            %                     end
                            try
                                z_new_rewire = replicate_with{bb};
                                [rewired,T,Graph,Edges,x_rewire,pnodes,pedges,added_new_rewire,idx_last_added_rewire] = ReWire(idx_near_bubble, idx_min, idx_new, z_new_rewire, T, Graph, Edges, Obstacles, Ptree,idx_prim, q, cost_new,plot_nodes,plot_edges,fig_xv,verbose);
                            catch ME
                                disp(ME.message);
                            end
                            plot_edges=pedges;
                            plot_nodes=pnodes;
                            
                            if ~isnan(idx_last_added_rewire)
                                idx_last_added = idx_last_added_rewire;
                            end
                        end
                    catch MI
                        disp(MI.message)
                        keyboard
                    end
                    
                    if checkdiscontinuity(T,Edges,Ptree)
                        %-keyboard
                    end
                    if added_new && rewired % && any(~any(isnan(x_rewire)))
                        %                     traj_pos_rewire=x_rewire(1,:);
                        %                     traj_vel_rewire=x_rewire(2,:);
                        %                     traj_yp_rewire =x_rewire(3,:);
                        %                     traj_pos = traj_pos_rewire;
                        %                     traj_vel = traj_vel_rewire;
                        %                     traj_y = traj_yp_rewire; % TODO: FIX NAMES
                        %                     x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)']; % assign arc-path
                        % this should fix the discontinuity problem
                        z_new = round(x_rewire(:,1)*100)/100;
                        %                     [z_new,x] = truncate_to_similar(z_new,x);
                        if checkdiscontinuity(T,Edges,Ptree)
                            %-keyboard
                        end
                    end
                    %                 %-keyboard % Uncomment here for tuning radius
                    if verbose
                        set(cerchio,'Visible','off')
                    end
                end
            else % not feasible as no near point has been found inside the search volume
                disp('Near set is empty');
                feasible = false;
            end
            
            if added_new
                if idx_prim ==1
                    if verbose
                        disp(['Found primitive ' prim.getName ' with cost: ' num2str(cost)]);
                    end
                else
                    if verbose
                        disp(['Found primitive ' prim.getName ' with cost: ' num2str(cost)]);
                    end
                end
            else
                if verbose
                    disp('No primitives found')
                end
            end
            
            %         if size(Edges,1) ~= size(Edges,2) || size(Graph,1) ~= size(Graph,2), disp('size issue 1:'),%-keyboard, end
            disp('### end')
        end
    else
        cprintf('error','Random and Nearest are not connectable with the primitive %s\n',prim.getName);
        % do nothing
    end
    
    Tree = T; % update tree
    G = Graph; % update graph
    E = Edges; % update edges
    
    if checkdiscontinuity(T,Edges,Ptree)
        %-keyboard
    end
    
    % % check se ha aggiunto il nodo giusto
    % if feasible && added_new %&& ~rewired
    %     % Extend the z_new point (already in the tree) with its initial_extend
    %     % values (see PrimitiveFun.extend)
    %     z_whatwas = z_new;
    %     z_new_temp=prim.extend(z_new);
    %     z_new_extended = fix_nans(z_new_temp,prim.dimensions);
    %     if checkdiscontinuity(T,E,Ptree)
    %         %-keyboard
    %     end
    %     before = T.get(T.nnodes);
    %     if T.nnodes<2
    %         disp('albero con un solo nodo')
    %         disp('# quitting localRRTstar #')
    %         return
    %     end
    %     before_E = E{T.Parent(T.nnodes),T.nnodes};
    %     before_T = T;
    %     if before(1:2) ~= z_whatwas(1:2)
    %         disp('ah-ah!')
    %         %-keyboard
    %     end
    %     T.Node{T.nnodes} = z_new_extended;
    %     after = T.get(T.nnodes);
    %     after_E = E{T.Parent(T.nnodes),T.nnodes};
    %     if checkdiscontinuity(T,E,Ptree)
    %         %-keyboard
    %     end
    % end
end
cprintf('*[0,0.7,1]*','# quitting localRRTstar #\n');