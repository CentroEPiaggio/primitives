function [T,G,E] = localRRTstar(Chi,Ptree,x_rand,T,Graph,Edges,Obstacles,verbose)

prim_cost = Inf(Ptree.nnodes,1);      % cost vector, to choose between different primitives the cheaper one
prim_feasible = zeros(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
prim_params = cell(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
actions = cell(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
fig_points=2;
fig_trajectories=3;

%% check if other dimensions can be activated from the newest point (x_rand)
for jj=1:1%Ptree.nnodes                       % start looking between all available primitives
    %     jj
    prim = Ptree.get(jj);                   % prim is the current primitive
    %% find the nearest point, in Chi0
    % convert nodes in trees from cells to matrix
    %         keyboard
    points_mat = cell2mat(T.Node');
    % now since we want to search in Chi0 we can remove all rows from
    % points_mat that does contain a NaN
    %     points_mat(~any(isnan(points_mat)),:)=[]
    points_mat(isnan(points_mat)) = []; % remove NaN from points
    points_mat = reshape(points_mat,2,size(T.Node,1)); % HARDFIX here 2 should be parametrized
    % find nearest pointisp
    %     keyboard
    idx_nearest = knnsearch(points_mat',x_rand'); % TODO: this can be replaced by a search in the least-cost sense, employing the primitive's cost_table
    x_nearest = T.get(idx_nearest);
    x_nearest(isnan(x_nearest)) = []; % remove NaN
    %     keyboard
    % nearest point found
    x_rand_temp=x_rand;
    x_nearest_temp=x_nearest;
    %     keyboard
    % TODO: how to represent the space? I suggest sth like sparse matrices
    % with NaNs for non-intersecting (or not yet sampled) dimensions.
    % UPDATE: this is how it's being implemented right now
    %     keyboard
    %waitforbuttonpress
    % TODO: valutare il miglior parametro per muoversi in Chi0
    dimChi0 = Chi.P.Dim;
    dimP = prim.chi.P.Dim;
    
    if dimP > dimChi0 % add trailing zeros for dimensions outside Chi0
        x_rand_temp(dimChi0+1:dimP) = 0.0;
        x_nearest_temp(dimChi0+1:dimP) = 0.0;
    end
    if prim.chi.P.contains([x_rand_temp, x_nearest_temp])
        %             prim_cost(jj) = searchCost(prim.cost_table); % now we can use
        %             the steering function
        xi = x_nearest_temp(1); vi = x_nearest_temp(2);
        xf = x_rand_temp(1); vf = x_rand_temp(2);
        q = [xi xf vi vf];
        [feasible,cost,q,traj_pos,traj_vel]=steering_muovi(xi,xf,vi,vf);
        x_new=x_rand;
        %         x_new(1) = q(2);
        %         x_new(2) = q(4);
        
        % collision checker loop
        % checks for feasibility of the returned steering function and, if
        % not feasible, looks for an intermediate point that might be able
        % to satisfy feasibility in the sense of no collisions along the
        % path
        if feasible
            kk=1;
            nested_feasible = false; nested_cost = Inf; nested_q = q; nested_traj_pos = traj_pos; nested_traj_vel = traj_vel;
            while any(Obstacles.Node{1}.P.contains([nested_traj_pos(:)'; nested_traj_vel(:)'])) && kk<=10 % tries only 10 times to shring the trajectory
                % ~nested_feasible &&
                kk = kk+1;
                q = [xi xf vi vf];
                start = [xi,vi];
                fin = [xf,vf];
                alpha = 1-0.1*kk; % decreases of 10 percent along the line connecting initial and final point
                point = (1-alpha)*start+(alpha)*fin;
                [nested_feasible,nested_cost,nested_q,nested_traj_pos,nested_traj_vel]=steering_muovi(xi,point(1),vi,point(2));
                %                 x_new=x_rand;
                x_new(1) = nested_traj_pos(end);%q(2);
                x_new(2) = nested_traj_vel(end);%q(4);
                %                 keyboard
                feasible = nested_feasible;
                cost = nested_cost;
                q = nested_q;
                traj_pos = nested_traj_pos;
                traj_vel = nested_traj_vel;
            end
        end
        
        if feasible
            %             %% TODO: add RRT* stuff
            %             % - Find nearby vertices (n meaning is not clear)
            %             n = 10;
            % Check for nearest point inside a certain bubble
            cardV = T.nnodes; % number of vertices in the graph
            [idx_near_bubble,raggio] = near(T,Graph,Edges,x_new,cardV);
            disp('###')
            idx_near_bubble % get all the nodes with T.get(idx_near_bubble{1})
            if raggio>0
                centro = x_new-raggio;
                diameter = 2*raggio;
                % Draw a circle around the nearest neighbors inside the bubble.
                %%
                figure(fig_points)
                cerchio = rectangle('position',[centro',diameter,diameter],...
                    'curvature',[1 1],'EdgeColor','b'); % 'LineStyle',':'
%                 figure(fig_trajectories)
%                 cerchio = rectangle('position',[centro',diameter,diameter],...
%                     'curvature',[1 1],'EdgeColor','b','LineStyle',':'); %
%                     'LineStyle',':'\

                keyboard
                set(cerchio,'visible','off'); % comment this if you want to keep all the circles on

% Find the parent with the lower cost
cost_from_x_nearest_to_new = cost;

[idx_min,q,cost_new] = ChooseParent(idx_near_bubble, idx_nearest, T, Graph, Edges, x_new,cost_from_x_nearest_to_new,Obstacles);

[T,Graph,Edges] = InsertNode(T.get(idx_min), x_new, idx_min, T, Graph, Edges, prim, q, cost_new);
keyboard
% [T,Graph,Edges] = ReWire(T.get(idx_min), x_new, idx_min, T, Graph, Edges, prim, q, cost_new);
[T,Graph,Edges] = ReWire(idx_near_bubble, idx_min, x_new, T, Graph, Edges, prim, q, cost_new);
            end
            disp('###')
%             keyboard
%             %% TEST Usage rangesearch
%             X=[1 1; 2 2;3 3;4 4]
%             Y=[2.1,2.1]
%             idX_near = rangesearch(X,Y,.1*sqrt(2))
%             idX_near = rangesearch(X,Y,.1001*sqrt(2))
            %%
            %             idX_near = rangesearch(points_mat',x_new',n); % QUESTO NON TORNA
            %             % - Choose Parent
            %             % the function can return either x_min or its ID
            %             idx_min = ChooseParent(idX_near, idx_nearest, T, Graph, x_new, cost);
            %             % x_min = T.get(idx_min); % get the value from index
            %             % - Insert Node
            %             x_min = T.get(idx_min);
            %             x_new = fix_nans(x_new,prim.dimensions);
            %             T = T.addnode(idx_min,x_new);
            %             % - Rewire
            % %             T = ReWire(T, idX_near, x_min, x_new);
            
        end
        
        %%
        figure(fig_trajectories)
        if feasible
            plot(traj_pos,traj_vel,'k');
        else
            plot(traj_pos,traj_vel,'r');
        end
        disp('In localRRTstar:')
        %         keyboard
        
        if feasible && all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'])) % after the AND we check if the trajectories go outside the primitive space (5th order polynomials are quite shitty)
            prim_feasible(jj) = feasible;
            prim_cost(jj) = cost;
            prim_params{jj} = q;
            if verbose
                disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(jj))]);
            end
            %% add the new node to the tree
            x_rand = fix_nans(x_rand,prim.dimensions);
            x_new = fix_nans(x_new,prim.dimensions);
            %             T = T.addnode(idx_nearest,x_rand);
            T = T.addnode(idx_nearest,x_new);% REMOVE THIS WHEN USING RRT* STUFF
            idx_last_added_node = length(T.Node);
            Graph(idx_nearest,idx_last_added_node) = cost;
            %             keyboard;
            
            actions{jj} = struct('source_node', idx_nearest,...
                'dest_node', idx_last_added_node,...
                'primitive',prim.name,...
                'primitive_q',q);
            
            % visualize tree-connection
            %             keyboard
            if verbose
                figure(fig_points)
                line([x_nearest(1) x_new(1)],[x_nearest(2) x_new(2)],'color','red','linewidth',2); % just for visualization
                %             line([x_nearest(1) x_rand(1)],[x_nearest(2) x_rand(2)],'color','black'); % just for visualization
                plot(x_new(1),x_new(2),'mx','linewidth',2)
                % visualize path in image space
                figure(fig_trajectories)
                plot(x_new(1),x_new(2),'mx','linewidth',2)
                plot(traj_pos,traj_vel);
            end
            %             keyboard
        else
            if verbose
                disp('No primitives found')
            end
            prim_feasible(jj) = 0;
            prim_cost(jj) = Inf;
            prim_params{jj} = 0;
            actions{jj} = NaN;
        end
    end
    
end

[~,idx_p_opt] = min(prim_cost);
if prim_cost(idx_p_opt) == Inf
    if verbose
        disp('nessuna primitiva con costo finito disponibile');
    end
    % elseif feasible
else
    prim_opt = Ptree.get(idx_p_opt);
    prim_params_opt = prim_params{idx_p_opt};
    %     keyboard
    idx_parent = actions{idx_p_opt}.source_node;
    idx_child  = actions{idx_p_opt}.dest_node;
    Edges{idx_parent,idx_child} = actions{idx_p_opt};
    if verbose
        disp(['scelgo la primitiva ' prim_opt.getName ' con un costo ' num2str(prim_cost(idx_p_opt)) ' e con parametro q=[' num2str(prim_params_opt) ']'])
    end
    % add the node and the edge to the graph
end
G = Graph; % update graph
E = Edges; % update edges
end
