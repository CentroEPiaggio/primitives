%INSERTNODE adds the new vertex z_new to the graph and creates an edge from
% z_current to z_new
% [T,G,E] = InsertNode(idx_current, z_new,  T, G, E, prim, q,cost)
function [added_node,T,G,E,... % inputs for algorithm stuff
    plot_nodes,plot_edges, ... % inputs for plotting visual stuff
    idx_last_added] ... % return index of last added node
    = InsertNode(idx_current, z_new,  T,     G,     E, prim, q,     cost, x, time, ... % inputs for algorithm stuff
    verbose, plot_nodes,plot_edges) % inputs for plotting visual stuff
disp('Entered inside InsertNode')
added_node = false;
if idx_current==0 % first node
    % add inial state to the tree
    T = T.addnode(0,z_new);
    added_node = true;
    idx_last_added = T.nnodes;
    return
else
    % %% debug init
    %     if ~isnan(x) & length(z_new)>2 & ~isequal(x(3,end), z_new(3,end)) % this check should not be necessary anymore thanks to rounding
    %         disp('sth strange is happening here')
    %         keyboard
    %         disp('forcing them to be equal')
    %         z_new_old = z_new;
    %         z_new(1:length(z_new)) = x(1:length(z_new),end);
    %         if (norm(z_new_old-z_new)) < 1e-9
    %             % do nothing
    %         else
    %             keyboard
    %         end
    %     end
    % %% debug end
    if ~isinf(cost) && cost>0
        test1 = fix_nans(T.Node{idx_current},prim.dimensions);
        test2 = fix_nans(z_new,prim.dimensions);
        
        if isequal(test1,test2) % length(test1)==length(test2) && all(test1==test2)
            cprintf('error','InsertNode: last node exception. Not adding the node.\n');
            keyboard
            return
        end
        z_new=fix_nans(z_new,prim.dimensions);
        %% this part makes sure that when attaching a new node which has a NaN dimension to a node which is non-NaN in the same dimension, the newly-attached node keeps the same non-NaN values (i.e. is put on the same plane)
        z_current = T.get(idx_current);
        %         keyboard
        %         z_new(~prim.dimensions) = x(~prim.dimensions,end); % this, executed after fix_nans, should fix the problem.
        % pseudo-code
        % se il nodo vecchio ha dimensioni diverse da prim.dimensions gia'
        % inizializzate, allora quelle dimensioni su z_new vanno messe a
        % x(~prim.dimensions,end). Altrimenti quelle stesse dimensioni vanno messe
        % come il vecchio nodo.
        % TODO BUGFIX: this fix probably works good for position variables, not for
        % speed variables.
        map_z_new_dimensions_not_initialized = isnan(z_new);
        map_z_current_dimensions_not_initialized = isnan(z_current);
        z_n = z_new;
        if ~isequal(map_z_current_dimensions_not_initialized,map_z_new_dimensions_not_initialized)
            z_n(map_z_new_dimensions_not_initialized>0) = x(map_z_new_dimensions_not_initialized>0,end);
        else
            z_n(map_z_new_dimensions_not_initialized>0) = z_current(map_z_new_dimensions_not_initialized>0,end);
        end
        z_new = z_n;
        if length(z_new) < 3
            keyboard
        end
        %
        if z_current(3) > 1 && z_new(3) == 1
            disp('Strange!!!!')
            keyboard
        end
        %
        %         z_new(~prim.dimensions) = current_node(~prim.dimensions);
        %
        
        
%         if any(any(isnan(x)))
%             disp('NaNs!!!!')
%             keyboard
%         end
        

        %
        
        %%
        try
            T = T.addnode(idx_current,z_new);
        catch
            keyboard;
        end
        idx_last_added_node = T.nnodes;
        % make the sparse matrix square % is this really needed here?
        % add new arc to the matrix: this was moved here, before assignig
        % zeros to null elements, to ensure that G is squared AFTER it has been modified
        G(idx_current,idx_last_added_node) = cost;
        sizeG = size(G);
        [~,shorterDim]=min(sizeG);
        G(sizeG(shorterDim)+1:max(sizeG),:)=0;
        actions = struct('source_node', idx_current,...
            'dest_node', idx_last_added_node,...
            'primitive',prim.name,...
            'primitive_ID',prim.ID,...
            'primitive_q',q,...
            'x',x,...
            'time',time);
        E{idx_current,idx_last_added_node} = actions;
        added_node = true;
        disp(['Using primitive: ' prim.name]);
        disp(['Added node: {' num2str(idx_last_added_node) '} as [' num2str(z_new(:)') ']. Parent: {' num2str(idx_current) '}.'])
        disp(['From node {' num2str(idx_current) '}: [' num2str(T.get(idx_current)') '] or {' num2str(E{idx_current,idx_last_added_node}.source_node) '}'])
        disp(['To   node {' num2str(idx_last_added_node) '}: [' num2str(T.get(idx_last_added_node)') '] or {' num2str(E{idx_current,idx_last_added_node}.dest_node) '}'])
        %%
        previous = T.get(T.Parent(idx_last_added_node));
        thisone = T.Node{idx_last_added_node};
        if ~isequaln(round(E{idx_current,idx_last_added_node}.x(3,end)*100)/100, round(T.Node{idx_last_added_node}(3)*100)/100) && ~isequaln(previous(3),thisone(3))
            disp('look at this shitface!')
            E{idx_current,idx_last_added_node}.x(3,end), T.Node{idx_last_added_node}(3)
            keyboard
        end
        
        if previous(3)>1 & thisone(3)==1
            disp('brought to ground floor... but how?')
            keyboard
        end
    else
        cprintf('error','InsertNode: cost is 0 or Inf.\n');
        keyboard
    end
end

%% Plotting tree in the state space % TODO: this has to be a method specific to each problem.
if verbose && added_node
    %     if isequal(prim.name,'Eleva')
    %         keyboard
    %     end
    fig_xv=2; fig_xy = 3; fig_yv = 4;
    figure(fig_xv)
    %                     z_min = z_nearest; % just for the next line, which is a visualization thing
    z_start = T.get(idx_current);
    
    if xor(isnan(z_start(3)),isnan(z_new(3)))
        keyboard
    end
    
    z_start_visual = z_start;
    z_new_visual = z_new;
%     if length(z_new_visual) == 2
%         z_new_visual(3) = 1; % HARDFIX: formally correct but it has to be generalized to the generic primitive/element
%     end
    if isnan(z_start_visual(5))
        z_start_visual(5) = 0; % HARDFIX: formally correct but it has to be generalized to the generic primitive/element
    end
    if isnan(z_new_visual(5))
        z_new_visual(5) = 0; % HARDFIX: formally correct but it has to be generalized to the generic primitive/element
    end
    node = plot(z_new_visual(1),z_new_visual(2),'bo','linewidth',2);
    plot_nodes = horzcat(plot_nodes,node);
    %                     keyboard
    edge = line([z_start(1) z_new_visual(1)],[z_start(2) z_new_visual(2)],'color',prim.edge_color,'linewidth',2);
    %                     keyboard
    %plot_edges = horzcat(plot_edges,edge);
    plot_edges{1,T.nnodes} = edge;
    %     keyboard
    figure(fig_xy)
    node = plot3(z_new_visual(1),z_new_visual(2),z_new_visual(3),'go','linewidth',2);
    %                         node = plot(z_new(1),z_new(3),'go','linewidth',2);
    plot_nodes = horzcat(plot_nodes,node);
    edge = line([z_start_visual(1) z_new_visual(1)],[z_start_visual(2) z_new_visual(2)],[z_start_visual(3) z_new_visual(3)],'color',prim.edge_color,'linewidth',2);
    %                         edge = line([z_min(1) z_new(1)],[z_min(3) z_new(3)],'color','green','linewidth',2);
    %                     plot_edges = horzcat(plot_edges,edge);
    plot_edges{2,T.nnodes} = edge;
end

idx_last_added = T.nnodes;