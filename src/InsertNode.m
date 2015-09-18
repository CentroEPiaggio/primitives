%INSERTNODE adds the new vertex z_new to the graph and creates an edge from
% z_current to z_new
% T = InsertNode(z_min, z_new, T, G, E)
function [T,G,E] = InsertNode(z_current, z_new, idx_current, T, G, E, prim, q,cost)
z_new=fix_nans(z_new,prim.dimensions);
T = T.addnode(idx_current,z_new);
idx_last_added_node = T.nnodes;
G(idx_current,idx_last_added_node) = cost;
% for now actions is not needed
actions = struct('source_node', idx_current,...
                'dest_node', idx_last_added_node,...
                'primitive',prim.name,...
                'primitive_q',q);
% E{idx_current,idx_new} = actions{idx_p_opt}