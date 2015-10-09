function [T,G,E,z_new,plot_nodes,plot_edges] = localRRTstar(Chi,Ptree,idx_prim,z_rand,T,Graph,Edges,Obstacles,verbose,plot_nodes,plot_edges)
prim_cost = Inf(Ptree.nnodes,1);      % cost vector, to choose between different primitives the cheaper one
prim_feasible = zeros(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
prim_params = cell(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
actions = cell(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
fig_xv=2; fig_xy = 3; fig_yv = 4;
%% check if other dimensions can be activated from the newest point (x_rand)
if idx_prim > 1
    keyboard
end
prim = Ptree.get(idx_prim);                   % prim is the current primitive
% search for nearest point
[idx_nearest,z_nearest] = nearest(z_rand,T);
z_min = z_nearest; % initialization of z_min which is the point in space that gives the lower cost
x_rand_temp=z_rand;
x_nearest_temp=z_nearest;

% TODO: valutare il miglior parametro per muoversi in Chi0
dimChi0 = Chi.P.Dim;
dimP = prim.chi.P.Dim;

if dimP > dimChi0 % add trailing zeros for dimensions outside Chi0
    x_rand_temp(dimChi0+1:dimP) = 0.0;
    x_nearest_temp(dimChi0+1:dimP) = 0.0;
end

if prim.chi.P.contains([x_rand_temp, x_nearest_temp],1) % check if both points are in the image space of the primitive
    xi = x_nearest_temp(1); vi = x_nearest_temp(2);
    xf = x_rand_temp(1); vf = x_rand_temp(2);
    q = [xi xf vi vf];
    [feasible,cost,q,traj_pos,traj_vel]=steering_muovi(xi,xf,vi,vf);
    z_new=z_rand;
    
    if feasible
        [feasible,cost,q,traj_pos,traj_vel]=CollisionFree(Obstacles,q,traj_pos,traj_vel,cost);
    end
    
    if feasible
        cardV = T.nnodes; % number of vertices in the graph
        [idx_near_bubble,raggio] = near(T,Graph,Edges,z_new,cardV);     % Check for nearest point inside a certain bubble
        disp('###')
        %             idx_near_bubble                                                 % get all the nodes with T.get(idx_near_bubble{1})
        if raggio>0
            %                 disp(['raggio: ' num2str(raggio)]);
            centro = z_new-raggio;
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
                [idx_min,q,cost_new,traj_pos_chooseparent,traj_vel_chooseparent] = ChooseParent(idx_near_bubble, idx_nearest, T, Graph, Edges, z_new,cost_from_z_nearest_to_new,Obstacles,q);
                if ~isnan(traj_pos_chooseparent)
                    traj_pos = traj_pos_chooseparent;
                    traj_vel = traj_vel_chooseparent;
                end
            end
            if all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'],1))
                [T,Graph,Edges] = InsertNode(idx_min, z_new, T, Graph, Edges, prim, q, cost_new);
                if verbose
                    figure(fig_xv)
                    node = plot(z_new(1),z_new(2),'bo','linewidth',2);
                    plot_nodes = horzcat(plot_nodes,node);
                    edge = line([z_min(1) z_new(1)],[z_min(2) z_new(2)],'color','blue','linewidth',2);
                    plot_edges = horzcat(plot_edges,edge);
                end
            end
            z_min = T.get(idx_min);
            
            idx_new = T.nnodes;
            [T,Graph,Edges,traj_pos_rewire,traj_vel_rewire,pn,pe] = ReWire(idx_near_bubble, idx_min, idx_new, T, Graph, Edges, Obstacles, prim, q, cost_new,plot_nodes,plot_edges,fig_xv);
            
            if ~isnan(traj_pos_rewire)
                traj_pos = traj_pos_rewire;
                traj_vel = traj_vel_rewire;
            end
            plot_edges=pe;
            plot_nodes=pn;
            if verbose
                set(cerchio,'Visible','off')
            end
        end
        disp('###')
    end
    
    if feasible && all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'],1)) % after the AND we check if the trajectories go outside the primitive space (5th order polynomials are quite shitty)
        prim_feasible(idx_prim) = feasible;
        prim_cost(idx_prim) = cost;
        prim_params{idx_prim} = q;
        if verbose
            disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(idx_prim))]);
        end
    else
        if verbose
            disp('No primitives found')
        end
        prim_feasible(idx_prim) = 0;
        prim_cost(idx_prim) = Inf;
        prim_params{idx_prim} = 0;
    end
end

G = Graph; % update graph
E = Edges; % update edges