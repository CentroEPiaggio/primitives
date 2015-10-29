function [T,G,E,z_new,plot_nodes,plot_edges,feasible] = localRRTstar(Chi,Ptree,idx_prim,z_rand,T,Graph,Edges,Obstacles,verbose,plot_nodes,plot_edges)
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

% % The next 6 lines of code allow a point to be extended with a coordinate
% % in the new primitive (e.g. a dimension from NaN becomes a real number)
% dimChi0 = Ptree.Node{1}.dimensions;%Chi.P.Dim;
% dimP = prim.dimensions; %.chi.P.Dim;
% dim_z_rand = ~isnan(z_rand);
% dim_z_nearest = ~isnan(z_nearest);

if all(prim.chi.P.contains([z_rand_temp(prim.dimensions>0), z_nearest_temp(prim.dimensions>0)],1)) % check if both points are in the image space of the primitive
    [feasible,cost,q,x,time] = prim.steering(z_nearest_temp,z_rand_temp); % uniform interface! Yeay!
    z_new=z_rand;
    dim_z_new = prim.dimensions;
    disp(['size durante la muovi',num2str(size(x))]);
    if feasible
        if idx_prim == 1 % collision checking only if we are on the Move primitive
            traj_pos = x(1,:);
            traj_vel = x(2,:);
            [feasible,cost,q,traj_pos,traj_vel]=CollisionFree(Obstacles,q,traj_pos,traj_vel,cost);
            if ~isnan(z_nearest_temp(3)) % HARDFIX
                traj_y   = z_nearest_temp(3,:)*ones(size(traj_vel));
            else
                traj_y   = ones(size(traj_vel)); % HARDFIX: default y is 1
            end
            x = [traj_pos; traj_vel;];
        else % Eleva primitive
            traj_vel = z_nearest_temp(2)*ones(size(x));%x(2,:);
            traj_pos = z_nearest_temp(1)+cumtrapz(time,traj_vel);
            traj_y = x;
            disp(['size durante la alza',num2str(size(x))]);
        end
        %                 keyboard
        x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)']; % assign arc-path % row vectors
    end
    
    if feasible
        cardV = T.nnodes; % number of vertices in the graph
        
        [idx_near_bubble,raggio] = near(T,Graph,Edges,z_new,dim_z_new,cardV);     % Check for nearest point inside a certain bubble
        disp('###')
        if raggio>0
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
                end
            end
            added_new = false;
            
            if idx_prim==1
                if true %all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'],1))
                    %                     keyboard
                    [T,Graph,Edges] = InsertNode(idx_min, z_new, T, Graph, Edges, prim, q, cost_new, x, time);
                    %                     if size(Edges,1) ~= size(Edges,2) || size(Graph,1) ~= size(Graph,2), disp('size issue 4:'),keyboard, end
                    added_new = true;
                end
            else % idx_prim > 1
                %                 keyboard
                if true %all(prim.chi.P.contains([traj_pos(:)'; traj_vel(:)'; traj_y(:)'],1))
                    %                     keyboard
                    [T,Graph,Edges] = InsertNode(idx_min, z_new, T, Graph, Edges, prim, q, cost_new, x, time);
                    %                     if size(Edges,1) ~= size(Edges,2) || size(Graph,1) ~= size(Graph,2), disp('size issue 3:'),keyboard, end
                    added_new = true;
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
            %             [T,Graph,Edges,traj_pos_rewire,traj_vel_rewire,pn,pe] = ReWire(idx_near_bubble, idx_min, idx_new, T, Graph, Edges, Obstacles, Ptree,idx_prim, q, cost_new,plot_nodes,plot_edges,fig_xv);
            [rewired,T,Graph,Edges,x_rewire,pn,pe] = ReWire(idx_near_bubble, idx_min, idx_new, T, Graph, Edges, Obstacles, Ptree,idx_prim, q, cost_new,plot_nodes,plot_edges,fig_xv);
            %             if rewired,keyboard,end
            if rewired && any(~any(isnan(x_rewire)))
                %                 if size(Edges,1) ~= size(Edges,2) || size(Graph,1) ~= size(Graph,2), disp('size issue 2:'),keyboard, end
                %                     keyboard
                traj_pos_rewire=x_rewire(1,:);
                traj_vel_rewire=x_rewire(2,:);
                traj_yp_rewire =x_rewire(3,:);
                traj_pos = traj_pos_rewire;
                traj_vel = traj_vel_rewire;
                traj_y = traj_yp_rewire; % TODO: FIX NAMES
                x = [traj_pos(:)'; traj_vel(:)'; traj_y(:)']; % assign arc-path
            end
            plot_edges=pe;
            plot_nodes=pn;
            %             end
            if verbose
                set(cerchio,'Visible','off')
            end
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
        %         if prim_cost(idx_prim) == 0 % PROBLEMA COSTO NULLO
        %             keyboard
        %         end
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

G = Graph; % update graph
E = Edges; % update edges