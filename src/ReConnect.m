%RECONNECT
% [T,G,E] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost)
function [T,G,E] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost)
z_near = T.get(idx_near);
% remove node
T = T.removenode(idx_near);
% re-insert the z_near node attaching it to the z_new node
[T,G,E] = InsertNode(idx_new, z_near, T, G, E, prim, q,cost);
