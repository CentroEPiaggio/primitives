%RECONNECT
% [T,G,E] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost)
function [T,G,E] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost)
z_near = T.get(idx_near);
z_near = fix_nans(z_near,prim.dimensions);
% re-insert the z_near node attaching it to the z_new node
% instead of InsertNode try to play with the parents of the node
% assigning a new parent to an existing node
idx_old = T.getparent(idx_near);
T.Parent(idx_near) = idx_new;
% remove old edge
E{idx_old,idx_near} = [];
G(idx_old,idx_near) = 0;

G(idx_new,idx_near) = cost;
actions = struct('source_node', idx_new,...
    'dest_node', idx_near,...
    'primitive',prim.name,...
    'primitive_q',q);
E{idx_new,idx_near} = actions;


