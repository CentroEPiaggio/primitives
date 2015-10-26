%INSERTNODE adds the new vertex z_new to the graph and creates an edge from
% z_current to z_new
% [T,G,E] = InsertNode(idx_current, z_new,  T, G, E, prim, q,cost)
function [T,G,E] = InsertNode(idx_current, z_new,  T, G, E, prim, q, cost, x,time)
disp('Entered inside InsertNode')
if isempty(prim)
    % add inial state to the tree
    T = T.addnode(0,z_new);
else
    test1 = fix_nans(T.Node{idx_current},prim.dimensions);
    test2 = fix_nans(z_new,prim.dimensions);
    
    if length(test1)==length(test2) && all(test1==test2)
        cprintf('error','InsertNode: last node exception. Not adding the node.\n');
        keyboard
        return
    end
    z_new=fix_nans(z_new,prim.dimensions);
    T = T.addnode(idx_current,z_new);
    idx_last_added_node = T.nnodes;
    % make the sparse matrix square % is this really needed here?
    sizeG = size(G);
    [~,shorterDim]=min(sizeG);
    G(sizeG(shorterDim)+1:max(sizeG),:)=0;
    % add new arc to the matrix
    G(idx_current,idx_last_added_node) = cost;
    %     keyboard
    actions = struct('source_node', idx_current,...
        'dest_node', idx_last_added_node,...
        'primitive',prim.name,...
        'primitive_q',q,...
        'x',x,...
        'time',time);
    E{idx_current,idx_last_added_node} = actions;
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