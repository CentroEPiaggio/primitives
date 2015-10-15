%NEAREST: find the nearest point in the tree to a given point
% [idx_nearest,z_nearest,dim_nearest] = nearest(x_rand,T)
% arguments:
% - x_rand: the given point
% - T:      the tree
% returns:
% - idx_nearest:    index of the nearest in the tree
% - z_nearest:      the nearest point
% - dim_nearest:    the dimension vector of the nearest point
% function [idx_nearest,z_nearest,dim_nearest] =
% nearest(z_rand,T,dimensions_z_rand,dimensions_chi0) % no need of
% dim_nearest, we can just keep the NaNs.
function [idx_nearest,z_nearest] = nearest(z_rand,T,dimensions_z_rand,dimensions_chi0)
% TODO: implement multiple-dimension distance calculator as in slides of
% planning_meeting_04
%% find the nearest point, in Chi0
% convert nodes in trees from cells to matrix
%keyboard
points_mat = cell2mat(T.Node');

% by definition, the closer nearest point has to be inside the same image
% space. I.e. no distance defined outside the primitive space.
if ~isequal(dimensions_chi0,dimensions_z_rand) % z_rand is an extended space
    % search for near inside the same image space
    points_mat_z_rand_dimension = points_mat(dimensions_z_rand>0,:);
    points_mat_z_rand_dimension = [points_mat_z_rand_dimension, [1;2;3]];
    points_mat_z_rand_dimension(:,any(isnan(points_mat_z_rand_dimension))) = [];
    % now points_mat_z_rand_dimension cointains only the points that live
    % in the same space of z_rand (i.e. have the same non-NaN dimensions)
    points_mat_z_rand_dimension = reshape(points_mat_z_rand_dimension,sum(dimensions_z_rand),size(points_mat_z_rand_dimension,2));
    idx_nearest = knnsearch(points_mat_z_rand_dimension',z_rand');
    z_nearest = T.get(idx_nearest);
else
    % if no nearest point has been found inside the image space of the point,
    % look for a close point in the Chi0 imagespace
    %
    % now since we want to search in Chi0 we can remove all rows from
    % points_mat that does contain a NaN
    %     points_mat(~any(isnan(points_mat)),:)=[]
    points_mat(isnan(points_mat)) = []; % remove NaN from points
%     points_mat = reshape(points_mat,sum(dimensions_chi0),size(T.Node,1));
points_mat = reshape(points_mat,sum(dimensions_chi0),length(points_mat)/sum(dimensions_chi0));
    % find nearest pointisp
    %     keyboard
    idx_nearest = knnsearch(points_mat',z_rand'); % TODO: this can be replaced by a search in the least-cost sense, employing the primitive's cost_table
    z_nearest = T.get(idx_nearest);
end

% no need of
% dim_nearest, we can just keep the NaNs.
% dim_nearest = isnan(z_nearest); % enabling bits for dimensions
% z_nearest(isnan(z_nearest)) = []; % remove NaN
