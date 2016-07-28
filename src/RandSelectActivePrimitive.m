% RandSelectActivePrimitive
% Gets as input a vector of zeros and ones, where
% 0: stands for unactive primitive
% 1: stands for active primitive
% and returns
% idx: the ID of the (uniformly) randomly selected active primitive where
% to sample from
function idx = RandSelectActivePrimitive(active_primitives)
idx_active = find(active_primitives>0);
idx = datasample(idx_active,1);
end