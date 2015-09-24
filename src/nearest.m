%NEAREST: find the nearest point in the tree to a given point
% [idx_nearest,z_nearest,dim_nearest] = nearest(x_rand,T)
% arguments:
% - x_rand: the given point
% - T:      the tree
% returns:
% - idx_nearest:    index of the nearest in the tree
% - z_nearest:      the nearest point
% - dim_nearest:    the dimension vector of the nearest point
function [idx_nearest,z_nearest,dim_nearest] = nearest(x_rand,T)
%% find the nearest point, in Chi0
% convert nodes in trees from cells to matrix
%         keyboard
points_mat = cell2mat(T.Node');
% now since we want to search in Chi0 we can remove all rows from
% points_mat that does contain a NaN
%     points_mat(~any(isnan(points_mat)),:)=[]
points_mat(isnan(points_mat)) = []; % remove NaN from points
points_mat = reshape(points_mat,2,size(T.Node,1)); % HARDFIX here 2 should be parametrized
% find nearest pointisp
%     keyboard
idx_nearest = knnsearch(points_mat',x_rand'); % TODO: this can be replaced by a search in the least-cost sense, employing the primitive's cost_table
z_nearest = T.get(idx_nearest);
dim_nearest = isnan(z_nearest); % enabling bits for dimensions
z_nearest(isnan(z_nearest)) = []; % remove NaN