function [T,G,E] = localRRTstar(Chi,Ptree,z_rand,T,Graph,Edges,Obstacles,verbose,printfigu)
cprintf('err','In localRRTstar:\n');
prim_cost = Inf(Ptree.nnodes,1);      % cost vector, to choose between different primitives the cheaper one
prim_feasible = zeros(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
prim_params = cell(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
actions = cell(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
fig_points=2;
fig_trajectories=3;

%% check if other dimensions can be activated from the newest point (x_rand)
for jj=1:1%Ptree.nnodes                       % start looking between all available primitives
    cprintf('cyan','checking primitive %d:\n',jj);
    %     jj
    prim = Ptree.get(jj);                   % prim is the current primitive
    
    cprintf('[1 0 1]','Nearest\n',jj);
    keyboard
    % search for nearest point
    [idx_nearest,z_nearest] = nearest(z_rand,T);
    z_min = z_nearest; % initialization of z_min which is the point in space that gives the lower cost
    x_rand_temp=z_rand;
    x_nearest_temp=z_nearest;
    if verbose && printfigu
        figure(fig_points);
        h_z_nearest=plot(z_nearest(1),z_nearest(2),'rx','linewidth',2);
        keyboard
        printafigu('figures/','fig_03');
        keyboard
        delete(h_z_nearest);
    end
    
    
    
    % TODO: valutare il miglior parametro per muoversi in Chi0
    dimChi0 = Chi.P.Dim;
    dimP = prim.chi.P.Dim;
    
    if dimP > dimChi0 % add trailing zeros for dimensions outside Chi0
        x_rand_temp(dimChi0+1:dimP) = 0.0;
        x_nearest_temp(dimChi0+1:dimP) = 0.0;
    end
    
    if prim.chi.P.contains([x_rand_temp, x_nearest_temp]) % check if both points are in the image space of the primitive
        xi = x_nearest_temp(1); vi = x_nearest_temp(2);
        xf = x_rand_temp(1); vf = x_rand_temp(2);
        q = [xi xf vi vf];
        cprintf('[1 0.5 0]','steering function: %s\n',prim.name);
        [feasible,cost,q,traj_pos,traj_vel]=steering_muovi(xi,xf,vi,vf);
        x_new=z_rand;
        if verbose && printfigu
            h_traj=plot(traj_pos,traj_vel);
            printafigu('figures/','fig_04');
            keyboard
        end
        cprintf('-comment','Collision checking.\n');
        if feasible
            [feasible,cost,q,traj_pos,traj_vel]=CollisionFree(Obstacles,q,traj_pos,traj_vel,cost);
            if verbose && printfigu
                h_traj_collisionfree=plot(traj_pos,traj_vel,'c','Linewidth',2);
                printafigu('figures/','fig_05');
                keyboard
                delete(h_traj_collisionfree);
            end
        end
        
        if feasible
            cardV = T.nnodes; % number of vertices in the graph
            [idx_near_bubble,raggio] = near(T,Graph,Edges,x_new,cardV);     % Check for nearest point inside a certain bubble
            disp('###')
            idx_near_bubble                                                 % get all the nodes with T.get(idx_near_bubble{1})
            keyboard
            
            if raggio>0
                disp(['raggio: ' num2str(raggio)]);
                keyboard
                centro = x_new-raggio;
                diameter = 2*raggio;
                figure(fig_points)
                cerchio = rectangle('position',[centro',diameter,diameter],... % Draw a circle around the nearest neighbors inside the bubble.
                    'curvature',[1 1],'EdgeColor','b'); % 'LineStyle',':'
                set(cerchio,'visible','on')
                keyboard
                % Find the parent with the lower cost
                cost_from_z_nearest_to_new = cost;
                disp('Entra in ChooseParent')
                if isempty(idx_near_bubble)                                 % if there is no near vertex in the bubble keep the nearest node and proceed to insert it in the tree
                    idx_min = idx_nearest;
                    cost_new = cost_from_z_nearest_to_new;
                    keyboard
                else                                                        % otherwise look for possibly more convenient paths
                    [idx_min,q,cost_new,traj_pos_chooseparent,traj_vel_chooseparent] = ChooseParent(idx_near_bubble, idx_nearest, T, Graph, Edges, x_new,cost_from_z_nearest_to_new,Obstacles,q);
                    if ~isnan(traj_pos_chooseparent)
                        traj_pos = traj_pos_chooseparent;
                        traj_vel = traj_vel_chooseparent;
                    end
                    h_traj_min_chooseparent = plot(traj_pos,traj_vel,'y','linewidth',2);
                    keyboard
                    delete(h_traj_min_chooseparent);
                end
                if all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)']))
                    disp('Entra in InsertNode')
                    if verbose
                        disp('Check here if the nodes were correctly inserted: before insertion')
                    keyboard
                    end
                    [T,Graph,Edges] = InsertNode(idx_min, x_new, T, Graph, Edges, prim, q, cost_new);
                    if verbose
                        disp('Check here if the nodes were correctly inserted: after insertion')
                    keyboard
                    end
                end
                if verbose
                    figure(fig_points)
                    b2 = plot(x_new(1),x_new(2),'bo','linewidth',2);
%                     b1 = line([z_min(1) x_new(1)],[z_min(2) x_new(2)],'color','blue','linewidth',2); 
                    plot(traj_pos,traj_vel,'b','linewidth',2);
                    if printfigu
                        printafigu('figures/','fig_06');
                        keyboard
                    end
                end
                idx_new = T.nnodes;
                disp('Entra in ReWire')
                disp('Prima di ReWire')
                keyboard
                [T,Graph,Edges,traj_pos_rewire,traj_vel_rewire] = ReWire(idx_near_bubble, idx_min, idx_new, T, Graph, Edges, Obstacles, prim, q, cost_new);
                disp('Dopo  di ReWire')
                if ~isnan(traj_pos_rewire)
                    traj_pos = traj_pos_rewire;
                    traj_vel = traj_vel_rewire;
                end
                if verbose
%                     points = [];
%                     for ii=1:length(T.Node)
%                         points = [points,ii];
%                         figure(fig_points)
%                         current_parent=T.Parent(ii);
%                         if current_parent~=0
%                             source = T.get(current_parent);
%                             source=fix_nans(source,prim.dimensions);
%                             goal = T.get(points(ii));
%                             goal=fix_nans(goal,prim.dimensions);
%                             a1=line([source(1) goal(1)],[source(2) goal(2)],'color','blue','linewidth',2);
%                             a2=plot(goal(1),goal(2),'bo','linewidth',2);
%                         end
%                     end
%                     set(a1,'Visible','off')
%                     set(a2,'Visible','off')
                    set(cerchio,'Visible','off')
                end
            end
            disp('###')
        end
        
        %% PLOTTING PART
        
        %         figure(fig_trajectories)
        %         if feasible
        %             plot(traj_pos,traj_vel,'k');
        %         else
        %             plot(traj_pos,traj_vel,'r');
        %         end
        
        if feasible && all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'])) % after the AND we check if the trajectories go outside the primitive space (5th order polynomials are quite shitty)
            prim_feasible(jj) = feasible;
            prim_cost(jj) = cost;
            prim_params{jj} = q;
            if verbose
                disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(jj))]);
            end
            
            %             if verbose
            %                 figure(fig_points)
            %                 line([z_min(1) x_new(1)],[z_min(2) x_new(2)],'color','red','linewidth',2); % just for visualization
            %                 plot(x_new(1),x_new(2),'mx','linewidth',2)
            %                 % visualize path in image space
            % %                 figure(fig_trajectories)
            % %                 plot(x_new(1),x_new(2),'mx','linewidth',2)
            % %                 plot(traj_pos,traj_vel);
            %             end
            
        else
            if verbose
                disp('No primitives found')
            end
            prim_feasible(jj) = 0;
            prim_cost(jj) = Inf;
            prim_params{jj} = 0;
            %             actions{jj} = NaN;
        end
    end
    
    G = Graph; % update graph
    E = Edges; % update edges
    
    %% b/w all the primitives choose the best one
    % [~,idx_p_opt] = min(prim_cost);
    % if prim_cost(idx_p_opt) == Inf
    %     if verbose
    %         disp('no primitives with a finite cost');
    %     end
    %     % elseif feasible
    % else
    %     prim_opt = Ptree.get(idx_p_opt);
    %     prim_params_opt = prim_params{idx_p_opt};
    %     %     keyboard
    % %     idx_parent = actions{idx_p_opt}.source_node;
    % %     idx_child  = actions{idx_p_opt}.dest_node;
    % %     Edges{idx_parent,idx_child} = actions{idx_p_opt};
    %     if verbose
    %         disp(['scelgo la primitiva ' prim_opt.getName ' con un costo ' num2str(prim_cost(idx_p_opt)) ' e con parametro q=[' num2str(prim_params_opt) ']'])
    %     end
    %     % add the node and the edge to the graph
    % end
    
end
