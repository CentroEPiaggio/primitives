%INSERTNODE adds the new vertex z_new to the graph and creates an edge from
% z_current to z_new
% [T,G,E] = InsertNode(idx_current, z_new,  T, G, E, prim, q,cost)
function [T,G,E] = InsertNode(idx_current, z_new,  T, G, E, prim, q, cost)
if isempty(prim)
    % add inial state to the tree
    T = T.addnode(0,z_new);
else
    z_new=fix_nans(z_new,prim.dimensions);
    T = T.addnode(idx_current,z_new);
    idx_last_added_node = T.nnodes;
    G(idx_current,idx_last_added_node) = cost;
    % for now actions is not needed
    actions = struct('source_node', idx_current,...
        'dest_node', idx_last_added_node,...
        'primitive',prim.name,...
        'primitive_q',q);
    E{idx_current,idx_last_added_node} = actions; % For now this is done at the
end