function [Tree,G,E,z_new,plot_nodes,plot_edges,feasible] = localRRTstar(Chi,Ptree,idx_prim,z_rand,T,Graph,Edges,Obstacles,verbose,plot_nodes,plot_edges)
prim_cost = Inf(Ptree.nnodes,1);      % cost vector, to choose between different primitives the cheaper one
prim_feasible = zeros(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
prim_params = cell(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
actions = cell(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
fig_xv=2; fig_xy = 3; fig_yv = 4;
feasible = 0;

%% check if other dimensions can be activated from the newest point (x_rand)
prim = Ptree.get(idx_prim);                   % prim is the current primitive
%% TODO FIX THIS Ptree.Node{idx_prim} instead of Ptree.Node{1}
[idx_nearest,z_nearest] = nearest(z_rand,T,Ptree.Node{idx_prim});

z_min = z_nearest; % initialization of z_min which is the point in space that gives the lower cost

z_rand_temp=z_rand; % why the heck did we define this temp vars?
z_nearest_temp=z_nearest;

z_new=z_rand;

if checkdiscontinuity(T,Edges,Ptree)
    disp('discontinuity!')
    %     keyboard
end

if all(prim.chi.P.contains([z_rand_temp(prim.dimensions>0), z_nearest_temp(prim.dimensions>0)],1)) % check if both points are in the image space of the primitive
    [feasible,cost,q,x,time] = prim.steering(z_nearest_temp,z_rand_temp); % uniform interface! Yeay!
    dim_z_new = prim.dimensions;
    if feasible
        % collision checking
        if idx_prim == 1
            %             [feasible,cost,q,traj_pos,traj_vel]=CollisionFree(Obstacles,q,traj_pos,traj_vel,cost);
            %             [feasible,cost,q,x,time]=CollisionFree(prim,Obstacles,q,x,time,z_nearest_temp,cost);
            traj_pos = x(1,:);
            traj_vel = x(2,:);
            if ~isnan(z_nearest_temp(3)) % HARDFIX
                traj_y   = z_nearest_temp(3,:)*ones(1,size(traj_vel,2));
            else
                traj_y   = ones(1,size(traj_vel,2)); % HARDFIX: default y is 1
            end
        else % Eleva primitive
            %             [feasible,cost,q,x,time]=CollisionFree(prim,Obstacles,q,x,time,z_nearest_temp,cost);
            traj_vel = z_nearest_temp(2)*ones(1,size(x,2));%x(2,:);
            traj_pos = z_nearest_temp(1)+cumtrapz(time,traj_vel);
            if size(x,1)>2
                traj_y = x(3,:);
            elseif size(x,1)==1
                traj_y = x;
            else
                disp('omg')
                keyboard
            end
        end
        %         keyboard
        x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)']; % assign arc-path % row vectors
        disp('before collisionfree')
        [feasible,cost,q,x,time]=CollisionFree(prim,Obstacles,q,x,time,z_nearest_temp,cost);
        disp('after  collisionfree')
        if checkdiscontinuity(T,Edges,Ptree)
            keyboard
        end
        % this should fix the discontinuity problem
        for jj=1:length(z_new)
            if ~isnan(z_new(jj))
                z_new(jj) = x(jj,end);
            end
        end
    end
    if checkdiscontinuity(T,Edges,Ptree)
        keyboard
    end
    if feasible && ~isequal(z_nearest_temp(1:2),x(1:2,1))
        disp('WTF?')
        keyboard
    end
    if feasible
        cardV = T.nnodes; % number of vertices in the graph
        
        [idx_near_bubble,raggio] = near(T,Graph,Edges,z_new,dim_z_new,cardV);     % Check for nearest point inside a certain bubble
        disp('###')
        if raggio>0 && ~isempty(idx_near_bubble) % 
            centro = z_new(1:2)-raggio;
            diameter = 2*raggio;
            figure(fig_xv)
            cerchio = rectangle('position',[centro',diameter,diameter],... % Draw a circle around the nearest neighbors inside the bubble.
                'curvature',[1 1],'EdgeColor','b'); % 'LineStyle',':'
            set(cerchio,'visible','on')
            
            % Find the parent with the lower cost
            cost_from_z_nearest_to_new = cost;
            if isempty(idx_near_bubble)                                 % if there is no near vertex in the bubble keep the nearest node and proceed to insert it in the tree
                idx_min = idx_nearest;
                cost_new = cost_from_z_nearest_to_new;
            else                                                        % otherwise look for possibly more convenient paths
                [idx_min,q,cost_new,x_chooseparent,time_chooseparent] = ChooseParent(idx_near_bubble, idx_nearest, T, Graph, Edges, z_new,cost_from_z_nearest_to_new,Obstacles,q,Ptree,idx_prim);
                %                 keyboard
                if ~isnan(x_chooseparent)
                    %                     keyboard
                    traj_pos_chooseparent=x_chooseparent(1,:);
                    traj_vel_chooseparent=x_chooseparent(2,:);
                    traj_yp_chooseparent =x_chooseparent(3,:);
                    traj_pos = traj_pos_chooseparent;
                    traj_vel = traj_vel_chooseparent;
                    traj_y = traj_yp_chooseparent; % TODO: FIX NAMES
                    x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)']; % assign arc-path
                    %                     if size(Edges,1) ~= size(Edges,2) || size(Graph,1) ~= size(Graph,2), disp('size issue 5:'),keyboard, end
                    if x(1:2,end) ~= z_new(1:2)
                        disp('ChooseParent slightly changed the goal point!')
%                         keyboard
                        %                     % this should fix the discontinuity problem
                        for jj=1:length(z_new)
                            if ~isnan(z_new(jj))
                                z_new(jj) = x(jj,end);
                            end
                        end
                    end
                    
                end
                tempnode = T.get(idx_min);
                if feasible && ~isequal(tempnode(1:2),x(1:2,1))
                    disp('WTF ChooseParent?')
                    keyboard
                end
            end
            
            if checkdiscontinuity(T,Edges,Ptree)
                keyboard
            end
            
            added_new = false;
            
            if idx_prim==1
                if all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'],1))
                    %                     keyboard
                    if z_new(1:2) ~= x(1:2,end)
                        disp('what???')
                        keyboard
                    end
                    [T,Graph,Edges] = InsertNode(idx_min, z_new, T, Graph, Edges, prim, q, cost_new, x, time);
                    %                     if size(Edges,1) ~= size(Edges,2) || size(Graph,1) ~= size(Graph,2), disp('size issue 4:'),keyboard, end
                    added_new = true;
                end
            else % idx_prim > 1
                %                 keyboard
                if all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'; traj_y(:)'],1))
                    %                     keyboard
                    if z_new(1:2) ~= x(1:2,end)
                        disp('what???')
                        keyboard
                    end
                    [T,Graph,Edges] = InsertNode(idx_min, z_new, T, Graph, Edges, prim, q, cost_new, x, time);
                    %                     if size(Edges,1) ~= size(Edges,2) || size(Graph,1) ~= size(Graph,2), disp('size issue 3:'),keyboard, end
                    added_new = true;
                end
            end
            if checkdiscontinuity(T,Edges,Ptree)
                keyboard
            end
            if added_new
                if checkdiscontinuity(T,Edges,Ptree)
                    keyboard
                end
                
            end
            if verbose && added_new
                figure(fig_xv)
                z_min_visual = z_min;
                z_new_visual = z_new;
                if length(z_new_visual) == 2
                    z_new_visual(3) = 1; % HARDFIX: formally correct but it has to be generalized to the generic primitive/element
                end
                if isnan(z_min_visual(3))
                    z_min_visual(3) = 1; % HARDFIX: formally correct but it has to be generalized to the generic primitive/element
                end
                node = plot(z_new_visual(1),z_new_visual(2),'bo','linewidth',2);
                plot_nodes = horzcat(plot_nodes,node);
                edge = line([z_min(1) z_new_visual(1)],[z_min(2) z_new_visual(2)],'color','blue','linewidth',2);
                plot_edges = horzcat(plot_edges,edge);
                figure(fig_xy)
                %                 keyboard
                node = plot3(z_new_visual(1),z_new_visual(2),z_new_visual(3),'go','linewidth',2);
                %                         node = plot(z_new(1),z_new(3),'go','linewidth',2);
                plot_nodes = horzcat(plot_nodes,node);
                edge = line([z_min_visual(1) z_new_visual(1)],[z_min_visual(2) z_new_visual(2)],[z_min_visual(3) z_new_visual(3)],'color','green','linewidth',2);
                %                         edge = line([z_min(1) z_new(1)],[z_min(3) z_new(3)],'color','green','linewidth',2);
                plot_edges = horzcat(plot_edges,edge);
            end
            
            z_min = T.get(idx_min);
            
            idx_new = T.nnodes;
            [rewired,T,Graph,Edges,x_rewire,pn,pe] = ReWire(idx_near_bubble, idx_min, idx_new, T, Graph, Edges, Obstacles, Ptree,idx_prim, q, cost_new,plot_nodes,plot_edges,fig_xv);
            
            if checkdiscontinuity(T,Edges,Ptree)
                keyboard
            end
            if added_new && rewired % && any(~any(isnan(x_rewire)))
                traj_pos_rewire=x_rewire(1,:);
                traj_vel_rewire=x_rewire(2,:);
                traj_yp_rewire =x_rewire(3,:);
                traj_pos = traj_pos_rewire;
                traj_vel = traj_vel_rewire;
                traj_y = traj_yp_rewire; % TODO: FIX NAMES
                x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)']; % assign arc-path
                % this should fix the discontinuity problem
                for jj=1:length(z_new)
                    if ~isnan(z_new(jj))
                        z_new(jj) = x(jj,end);
                    end
                end
                if checkdiscontinuity(T,Edges,Ptree)
                    keyboard
                end
            end
            %             if feasible && ~isequal((1:2),x(1:2,1))
            %                 disp('WTF ReWire?')
            %                 keyboard
            %             end
            plot_edges=pe;
            plot_nodes=pn;
            %             end
            if verbose
                set(cerchio,'Visible','off')
            end
        else % not feasible as no near point has been found inside the search volume
            feasible = false;
        end
        %         if size(Edges,1) ~= size(Edges,2) || size(Graph,1) ~= size(Graph,2), disp('size issue 1:'),keyboard, end
        disp('###')
    end
    
    if feasible
        if idx_prim ==1
            if all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'],1)) % after the AND we check if the trajectories go outside the primitive space (5th order polynomials are quite shitty)
                prim_feasible(idx_prim) = feasible;
                prim_cost(idx_prim) = cost;
                prim_params{idx_prim} = q;
                if verbose
                    disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(idx_prim))]);
                end
            end
        else
            if all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'; traj_y(:)'],1)) % after the AND we check if the trajectories go outside the primitive space (5th order polynomials are quite shitty)
                prim_feasible(idx_prim) = feasible;
                prim_cost(idx_prim) = cost;
                prim_params{idx_prim} = q;
                if verbose
                    disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(idx_prim))]);
                end
            end
        end
    else
        if verbose
            disp('No primitives found')
        end
        prim_feasible(idx_prim) = 0;
        prim_cost(idx_prim) = Inf;
        prim_params{idx_prim} = 0;
    end
    
else
    cprintf('error','Random and Nearest are not connectable with the primitive %s\n',prim.getName);
    % do nothing
end

Tree = T; % update tree
G = Graph; % update graph
E = Edges; % update edges

if checkdiscontinuity(T,Edges,Ptree)
    keyboard
end

% check se ha aggiunto il nodo giusto
if feasible && ~rewired
    % Extend the z_new point (already in the tree) with its initial_extend
    % values (see PrimitiveFun.extend)
    z_whatwas = z_new;
    z_new_temp=prim.extend(z_new);
    z_new_extended = fix_nans(z_new_temp,prim.dimensions);
    if checkdiscontinuity(T,E,Ptree)
        keyboard
    end
    before = T.get(T.nnodes)
    if T.nnodes<2
        disp('albero con un solo nodo')
        return
    end
    before_E = E{T.Parent(T.nnodes),T.nnodes};
    before_T = T;
    if before(1:2) ~= z_whatwas(1:2)
        disp('ah-ah!')
        keyboard
    end
    T.Node{T.nnodes} = z_new_extended;
    after = T.get(T.nnodes)
    after_E = E{T.Parent(T.nnodes),T.nnodes};
    if checkdiscontinuity(T,E,Ptree)
        keyboard
    end
end