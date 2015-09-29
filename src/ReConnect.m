%RECONNECT
% [T,G,E] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost)
function [T,G,E] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost)
z_near = T.get(idx_near);
z_near = fix_nans(z_near,prim.dimensions);
% re-insert the z_near node attaching it to the z_new node
%% TODO redesign this part:
% instead of InsertNode try to play with the parents of the node
% assigning a new parent to an existing node
[T,G,E] = InsertNode(idx_new, z_near, T, G, E, prim, q,cost);
% remove node
T = T.removenode(idx_near);

% update G and E
%% TODO

