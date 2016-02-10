%INSERTNODE adds the new vertex z_new to the graph and creates an edge from
% z_current to z_new
% [T,G,E] = InsertNode(idx_current, z_new,  T, G, E, prim, q,cost)
function [added_node,T,G,E] = InsertNode(idx_current, z_new,  T,     G,     E, prim, q,     cost, x, time)
disp('Entered inside InsertNode')
added_node = false;
if idx_current==0 % first node
    % add inial state to the tree
    T = T.addnode(0,z_new);
    added_node = true;
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
%         % add new arc to the matrix
%         G(idx_current,idx_last_added_node) = cost;
        %     keyboard
        actions = struct('source_node', idx_current,...
            'dest_node', idx_last_added_node,...
            'primitive',prim.name,...
            'primitive_q',q,...
            'x',x,...
            'time',time);
        E{idx_current,idx_last_added_node} = actions;
        added_node = true;
        disp(['Using primitive: ' prim.name]);
        disp(['Added node: {' num2str(idx_last_added_node) '} as [' num2str(z_new(:)') ']. Parent: {' num2str(idx_current) '}.'])
        disp(['From node {' num2str(idx_current) '}: [' num2str(T.get(idx_current)') '] or {' num2str(E{idx_current,idx_last_added_node}.source_node) '}'])
        disp(['To   node {' num2str(idx_last_added_node) '}: [' num2str(T.get(idx_last_added_node)') '] or {' num2str(E{idx_current,idx_last_added_node}.dest_node) '}'])
    else
        cprintf('error','InsertNode: cost is 0 or Inf.\n');
        keyboard
    end
end


%
% if T.nnodes>1
%     if sum(isnan(T.Node{end})) == sum(isnan(T.Node{end-1}))
%         if all(fix_nans(T.Node{end})==fix_nans(T.Node{end-1}))
%             disp('InsertNode is the worst!')
%             keyboard
%         end
%     end
% end