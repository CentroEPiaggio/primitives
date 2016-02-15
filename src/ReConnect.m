%RECONNECT
% [T,G,E] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost)
function [T,G,E,plot_nodes,plot_edges] = ReConnect(idx_new,idx_near,T,G,E,Ptree,prim_list,q_list,cost_list, x_list, time_list, z_list, plot_nodes,plot_edges,fig_points,verbose)
cprintf('*[0,0,0]*','>>> Enter ReConnect\n');
% if isequal(prim_list.name,'Eleva')
%     keyboard
% end
keyboard
z_near = T.get(idx_near);
% z_near = fix_nans(z_near,prim_list.dimensions); % WARNING: removed



idx_start_temp = idx_new;

for ii=1:length(prim_list)
    prim_temp = Ptree.get(prim_list{ii});
    if ii<length(prim_list) % Avanza coi Goal-Start fino a quando il goal non e' giunto sul near. Se sei su un punto intermedio, InsertNode. Altrimenti rimappa e basta.
        z_goal_temp = z_list{ii};
        [added_new,T,G,E,plot_nodes,plot_edges] = InsertNode(idx_start_temp, z_goal_temp, T, G, E, prim_temp, q_list{ii}, cost_list{ii}, x_list{ii}, time_list{ii}, verbose, plot_nodes, plot_edges);
        if added_new
            cprintf('*[0,0,0]*','ReConnect-ed adding an edge from node %d to a new intermediate node %d\n',idx_start_temp,T.nnodes);
            idx_start_temp = T.nnodes; % next iteration starts from the last added node
        else
            disp('Failed adding node during ReConnect!');
            keyboard
        end
    else %% we are adding now the latest node of the sequence
        idx_goal_temp = idx_near;
        
        cprintf('*[0,0,0]*','ReConnect-ed from new node %d to old node %d\n',idx_start_temp,idx_goal_temp);

        z_goal_temp = z_near;
        % re-insert the z_near node attaching it to the z_new node
        % instead of InsertNode try to play with the parents of the node
        % assigning a new parent to an existing node
        idx_temp_old_parent = T.getparent(idx_goal_temp);
        T.Parent(idx_goal_temp) = idx_start_temp;
        % remove old edge
        E{idx_temp_old_parent,idx_goal_temp} = [];
        % keyboard % use plot_biograph(1,T.nnodes,G) to visualize rewiring. Call it
        % here and after G(idx_new,idx_near) = cost;
        G(idx_temp_old_parent,idx_goal_temp) = 0;
        % set(pe(idx_near),'visible','off');
        % delete(pe(idx_near));
        delete(plot_edges{1,idx_goal_temp});
        % delete(pe{2,idx_near}); % TODO on vertical graph
        
        G(idx_start_temp,idx_goal_temp) = cost_list{ii};        
        
        actions = struct('source_node', idx_start_temp,...
            'dest_node', idx_goal_temp,...
            'primitive',prim_temp.name,...
            'primitive_q',q_list{ii},...
            'x',x_list{ii},...
            'time',time_list{ii});
        E{idx_start_temp,idx_goal_temp} = actions;
        
        z_start_temp = T.get(idx_start_temp);
        figure(fig_points)
        %node = plot(z_new(1),z_new(2),'bo','linewidth',2);
        %plot_nodes = horzcat(plot_nodes,node);
        edge = line([z_start_temp(1) z_goal_temp(1)],[z_start_temp(2) z_goal_temp(2)],'color','magenta','linewidth',2); % just useful when debugging
        delete(edge);
        edge = line([z_start_temp(1) z_goal_temp(1)],[z_start_temp(2) z_goal_temp(2)],'color',prim_temp.edge_color,'linewidth',2);
        plot_edges{1,idx_goal_temp}=edge; % THIS SEEMS TO BE RIGHT
    end

end

cprintf('*[0,0,0]*','ReConnect-ed from new node %d to old node %d\n',idx_new,idx_near);

if checkdiscontinuity(T,E,Ptree)
                keyboard
end
            
cprintf('*[0,0,0]*','<<< Exit ReConnect\n');